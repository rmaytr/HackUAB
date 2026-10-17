"""Offline starter tests. Run with: python -m pytest tests/test_fusion_starter.py -q"""
from __future__ import annotations
from dataclasses import dataclass
import sys
import types
import numpy as np
import pytest
from fusion.synthetic import make_frame
from fusion.schema import FrameBundle, FusedObject, FusionResult
from fusion.fuse import fuse_frame, project_to_image
from fusion.bridge import to_cone_detections, to_obstacles, inject_fusion


@pytest.fixture
def api_stubs(monkeypatch):
    # Only used for isolated bridge tests. In the HackUAB repo, import real
    # autocross.api classes through simulator/.venv instead of these stubs.
    api = types.ModuleType("autocross.api")
    autocross = types.ModuleType("autocross")

    @dataclass
    class ConeDetections:
        positions: np.ndarray
        colors: np.ndarray
        range_m: float
        fov_rad: float

    @dataclass
    class ObstacleInfo:
        distance: float
        lateral_min: float
        lateral_max: float
        center: np.ndarray
        points: np.ndarray

    @dataclass
    class RoadModel:
        valid: bool
        range_m: float
        preview_s: np.ndarray
        preview_centerline: np.ndarray
        obstacles: list

    @dataclass
    class Observation:
        cones: object
        road: object

    api.ConeDetections = ConeDetections
    api.ObstacleInfo = ObstacleInfo
    api.RoadModel = RoadModel
    api.Observation = Observation
    autocross.api = api
    monkeypatch.setitem(sys.modules, "autocross", autocross)
    monkeypatch.setitem(sys.modules, "autocross.api", api)
    return api


def test_fusion_finds_synthetic_cones_with_correct_colors():
    frame, truth_xy, truth_colors = make_frame(0)
    fused = fuse_frame(frame)
    assert len(fused.objects) >= 10
    matches = 0
    for xy, c in zip(truth_xy, truth_colors):
        if any(np.linalg.norm(o.xy - xy) < .4 and o.color == int(c)
               for o in fused.objects):
            matches += 1
    assert matches >= 0.90 * len(truth_xy)
    assert any(o.color == 2 and "radar" in o.sources and np.isfinite(o.radial_velocity)
               for o in fused.objects)
    assert fused.stage_ms["total"] >= 0.


def test_missing_radar_is_safe():
    frame, _, _ = make_frame(0)
    frame.radar = np.empty((0,4),dtype=np.float32)
    objects = fuse_frame(frame).objects
    assert objects
    assert all(np.isnan(o.radial_velocity) for o in objects)
    assert all("radar" not in o.sources for o in objects)


def test_empty_lidar_is_safe():
    frame, _, _ = make_frame(0)
    frame.lidar = np.empty((0,4),dtype=np.float32)
    out = fuse_frame(frame)
    assert out.objects == [] and out.lidar_points_used == 0


def test_rgb_classification_does_not_invent_labels():
    frame, _, _ = make_frame(0)
    frame.image[:] = [54,55,57]
    objects = fuse_frame(frame).objects
    assert objects and all(o.color == -1 for o in objects)


def test_projection_sign_convention_left_is_left_pixel():
    frame, _, _ = make_frame(0)
    uv = project_to_image(np.array([[12., 3., .4], [12., -3., .4]]), frame)
    assert uv[0,0] < frame.K[0,2] < uv[1,0]


def test_frame_save_load(tmp_path):
    f, _, _ = make_frame(0)
    path = tmp_path/"frame.npz"
    f.save(path)
    reloaded = FrameBundle.load(path)
    assert reloaded.source == "synthetic"
    np.testing.assert_array_equal(reloaded.image, f.image)
    np.testing.assert_allclose(reloaded.lidar, f.lidar)
    np.testing.assert_allclose(reloaded.radar, f.radar)
    assert reloaded.t == f.t


def _fused(*objects):
    return FusionResult(list(objects), {}, 0, 0)


def _obj(x,y,c):
    return FusedObject(np.array([x,y],float),c,float('nan'),.9,25,("lidar","camera"))


def test_cone_bridge_respects_range_and_class(api_stubs):
    o = _fused(_obj(10,3,0), _obj(10,-3,1), _obj(12,.2,2), _obj(35,0,2), _obj(8,0,-1))
    cones = to_cone_detections(o, range_m=30)
    assert cones.positions.shape == (3,2)
    np.testing.assert_array_equal(cones.colors,[0,1,2])


def test_obstacle_bridge_uses_continuous_centerline_projection(api_stubs):
    # centerline turns left: y=5 at x=20; +left normal is not pure +y.
    road=api_stubs.RoadModel(True,40.,np.array([0.,10.,20.,30.]),
                             np.array([[0.,0.],[10.,0.],[20.,5.],[30.,10.]]),[])
    o=_fused(_obj(16.,4.,2),_obj(16.4,4.1,2),_obj(14.,-3.,0))
    obstacles=to_obstacles(o,road)
    assert len(obstacles)==1
    assert obstacles[0].points.shape == (2,2)
    assert 10. < obstacles[0].distance < 20.
    assert obstacles[0].lateral_min < obstacles[0].lateral_max
    # Semantics: orange clusters go to obstacles, blue boundary cones do not.
    assert len(obstacles[0].points) == 2


def test_inject_is_immutable_and_preserves_road_geometry(api_stubs):
    road=api_stubs.RoadModel(True,40.,np.linspace(0,40,41),
                             np.column_stack([np.linspace(0,40,41),np.zeros(41)]),[])
    obs=api_stubs.Observation(None,road)
    fused=_fused(_obj(18.,-.3,2),_obj(16.,3.6,0))
    changed=inject_fusion(obs,fused)
    assert changed is not obs
    assert changed.road is not road
    assert road.obstacles == []
    assert len(changed.road.obstacles)==1
    assert changed.cones.colors.tolist()==[2,0]
    assert changed.road.obstacles[0].distance==pytest.approx(18., abs=.1)


def test_carla_colors_are_unknown_until_semantically_verified():
    from fusion.bridge import label_from_road_prior
    frame, _, _ = make_frame(0)
    frame.source = "carla"  # fake this source to exercise the safety gate
    fused = fuse_frame(frame)
    assert fused.objects and all(o.color == -1 for o in fused.objects)
    s = np.linspace(0, 40, 41)
    road = types.SimpleNamespace(preview_s=s,
        preview_centerline=np.column_stack((s, np.zeros_like(s))),
        preview_width_left=np.full_like(s, 3.6),
        preview_width_right=np.full_like(s, 3.6),
        range_m=40.)
    recovered = label_from_road_prior(fused, road)
    assert any(o.color == 0 for o in recovered.objects)
    assert any(o.color == 1 for o in recovered.objects)
    assert any(o.color == 2 for o in recovered.objects)
    assert any("road_prior" in o.sources for o in recovered.objects)
