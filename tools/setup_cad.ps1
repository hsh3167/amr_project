$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    if (-not (Test-Path -LiteralPath '.venv/Scripts/python.exe')) {py -3.10 -m venv .venv}
    & '.venv/Scripts/python.exe' -m pip install --disable-pip-version-check --no-compile -r requirements-cad.txt
    if ($LASTEXITCODE -ne 0) {throw 'CAD environment setup failed.'}
    & '.venv/Scripts/python.exe' -c 'import build123d; print("build123d",build123d.__version__)'
} finally {Pop-Location}
