param(
    [string]$StepPath,
    [switch]$Headless,
    [switch]$Wait,
    [int]$TimeoutSeconds=600
)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
if (-not $StepPath) { $StepPath=Join-Path $projectRoot 'output/cad/amr_current.step' }
$stepResolved=(Resolve-Path -LiteralPath $StepPath).Path
$exeName=if ($Headless) {'freecadcmd.exe'} else {'freecad.exe'}
$candidates=@((Join-Path $env:ProgramFiles "FreeCAD 1.1/bin/$exeName"),(Join-Path $env:ProgramFiles "FreeCAD 1.0/bin/$exeName"))
$freecadExe=$candidates | Where-Object {Test-Path -LiteralPath $_} | Select-Object -First 1
if (-not $freecadExe) { $freecadExe=(Get-Command $exeName -ErrorAction SilentlyContinue).Source }
if (-not $freecadExe) { throw 'FreeCAD was not found. Install FreeCAD or put its bin folder on PATH.' }
$macro='cad/freecad/ImportAMR.FCMacro'
if ($Headless) {$macro='cad/freecad/import_amr.py'}
$reportDir=Join-Path $projectRoot 'output/freecad'
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
$env:AMR_CAD_STEP=$stepResolved
$env:AMR_SAVE_NATIVE='1'
$env:AMR_CAD_NATIVE=Join-Path $reportDir 'amr_review.FCStd'
$runId=[guid]::NewGuid().ToString('N')
$env:AMR_CAD_REVIEW_REPORT=Join-Path $reportDir "review_report_$runId.json"
$opts=@{FilePath=$freecadExe;ArgumentList=@('"'+$macro+'"');PassThru=$true;WorkingDirectory=$projectRoot}
if ($Headless) {
    $opts.WindowStyle='Hidden'
    $opts.RedirectStandardOutput=Join-Path $reportDir "freecad_${runId}_stdout.log"
    $opts.RedirectStandardError=Join-Path $reportDir "freecad_${runId}_stderr.log"
}
$reviewProcess=Start-Process @opts
if ($Wait) {
    $limit=(Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $limit -and -not (Test-Path -LiteralPath $env:AMR_CAD_REVIEW_REPORT) -and -not $reviewProcess.HasExited) {Start-Sleep -Milliseconds 250}
    if (-not (Test-Path -LiteralPath $env:AMR_CAD_REVIEW_REPORT)) {
        if ($Headless -and -not $reviewProcess.HasExited) {Stop-Process -Id $reviewProcess.Id}
        throw 'FreeCAD did not finish; inspect output/freecad logs.'
    }
    $state=Get-Content -LiteralPath $env:AMR_CAD_REVIEW_REPORT -Raw | ConvertFrom-Json
    if (-not $state.valid) {throw $state.error}
    Copy-Item -LiteralPath $env:AMR_CAD_REVIEW_REPORT -Destination (Join-Path $reportDir 'review_report.json') -Force
    if ($Headless -and -not $reviewProcess.HasExited) {Stop-Process -Id $reviewProcess.Id}
    [PSCustomObject]@{Valid=$state.valid;Solids=$state.solids;FCStd=$state.native}
} else { [PSCustomObject]@{ProcessId=$reviewProcess.Id;Report=$env:AMR_CAD_REVIEW_REPORT} }
