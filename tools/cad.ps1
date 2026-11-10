param([Parameter(ValueFromRemainingArguments=$true)][string[]]$CadArguments)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$pythonExe=Join-Path $projectRoot '.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) {throw 'Run tools/setup_cad.ps1 first.'}
Push-Location $projectRoot
try { & $pythonExe -m cad @CadArguments; if ($LASTEXITCODE -ne 0) {exit $LASTEXITCODE} }
finally {Pop-Location}
