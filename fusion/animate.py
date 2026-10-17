"""Generate a compact side-by-side GIF from synthetic fusion frames (Pillow only).

From HackUAB repo root:
  simulator/.venv/bin/python -m fusion.animate --frames 120 --stride 3 --out fusion_demo.gif
The GIF is an OFFLINE sensor demo, not a closed-loop lap or a CARLA recording.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
from .synthetic import make_frame
from .fuse import fuse_frame, project_to_image

COLORS={0:(66,142,255),1:(242,206,60),2:(254,125,42),-1:(180,180,185)}


def render(frame, fused):
    from PIL import Image, ImageDraw
    canvas=Image.new("RGB",(1000,495),(23,31,43))
    dr=ImageDraw.Draw(canvas)
    dr.text((24,16),"Camera + fused cone labels",fill=(232,239,245))
    dr.text((622,16),"LiDAR bird's-eye view / radar",fill=(232,239,245))
    camera=Image.fromarray(frame.image).resize((560,315),resample=Image.Resampling.NEAREST)
    canvas.paste(camera,(24,44))
    dr.rectangle((615,43,972,359),fill=(38,48,55),outline=(110,120,130),width=1)
    # Convert vehicle x-forward -> up, y-left -> left on the figure.
    def bev(xy):
        return (794 - int(float(xy[1])*24), 343 - int(float(xy[0])*8))
    for boundary in (-3.6,3.6):
        v=bev(np.array([0.,boundary]))[0]
        dr.line([(v,43),(v,359)],fill=(108,117,123),width=2)
    lidar=frame.lidar
    clean=lidar[(lidar[:,2]>.08)&(lidar[:,2]<1.5)]
    for pt in clean[::3]:
        px,py=bev(pt[:2])
        if 617<px<970 and 44<py<357:
            dr.point((px,py),fill=(116,137,148))
    for ob in fused.objects:
        xy=ob.xy
        xy3=np.array([[xy[0],xy[1],.43]])
        u,v=project_to_image(xy3,frame)[0]
        px,py=24+int(u*560/320),44+int(v*315/180)
        col=COLORS[ob.color]
        if 24<=px<584 and 44<=py<359:
            dr.ellipse((px-6,py-6,px+6,py+6),outline=col,width=2)
        bx,by=bev(xy)
        if 617<bx<970 and 44<by<357:
            dr.ellipse((bx-5,by-5,bx+5,by+5),fill=col,outline=(14,23,34))
            if np.isfinite(ob.radial_velocity):
                dr.text((bx+7,by-11),f"{ob.radial_velocity:+.1f}m/s",fill=(235,235,235))
    car=bev(np.array([0.,0.]))
    dr.polygon([(car[0],car[1]-11),(car[0]-9,car[1]+5),(car[0]+9,car[1]+5)],
               fill=(235,240,247))
    dr.text((24,378),f"Time {frame.t:.1f}s   synthetic frame    "
                      f"LiDAR pts {fused.lidar_points_used}    "
                      f"Fused objects {len(fused.objects)}",fill=(230,235,240))
    dr.text((24,404),f"Fusion CPU {fused.stage_ms['total']:.2f} ms/frame (rendering excluded)",
            fill=(102,231,204))
    dr.text((24,433),"Blue=left boundary   Yellow=right boundary   Orange=obstacle   Gray=unknown",
            fill=(190,204,215))
    dr.text((24,460),"Synthetic sensor demo: not CARLA evidence or scored controller output",
            fill=(162,174,184))
    return canvas


def main():
    p=argparse.ArgumentParser(description="Make 2-panel synthetic fusion GIF")
    p.add_argument("--frames",type=int,default=120)
    p.add_argument("--stride",type=int,default=3)
    p.add_argument("--out",default="fusion_demo.gif")
    a=p.parse_args()
    if a.frames<1 or a.stride<1:
        p.error("frames and stride must be positive")
    frames=[]
    for i in range(0,a.frames,a.stride):
        f,_,_=make_frame(i)
        frames.append(render(f,fuse_frame(f)))
    dst=Path(a.out); dst.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(dst,save_all=True,append_images=frames[1:],optimize=True,
                   duration=round(50*a.stride),loop=0)
    print(f"Saved {len(frames)} synthetic frames to {dst}")


if __name__=="__main__":
    main()
