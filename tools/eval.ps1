# Dev-only: run a controller on practice_technical + random seeded courses (bicycle, headless).
# From simulator/:   powershell -File ..\tools\eval.ps1 -Controller ..\tools\tracker_dev_controller.py
param(
    [string]$Controller = "..\hackuab\controller.py",
    [string]$Team = "dev",
    [string]$Seeds = "42,7,123",    # comma-separated; a string because -File collapses int[] args
    [string]$Config = ""            # optional --controller-config YAML (e.g. tracker overrides)
)
$extra = if ($Config) { @("--controller-config", $Config) } else { @() }
$py = ".venv\Scripts\python.exe"
New-Item -ItemType Directory -Force ..\tracks | Out-Null     # repo-root tracks/ is git-ignored
$courses = @("tracks\practice_technical.yaml")
foreach ($s in ($Seeds -split ",")) {
    $s = $s.Trim()
    $f = "..\tracks\seed$s.yaml"
    if (-not (Test-Path $f)) { & $py -m autocross gen-track --seed $s --obstacles 3 --out $f | Out-Null }
    $courses += $f
}
foreach ($c in $courses) {
    & $py -m autocross run -c $Controller -t $c --backend bicycle --no-render --team $Team @extra | Where-Object { $_ -match "===|cones|path err|technical" }
}
