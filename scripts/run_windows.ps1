param(
    [string]$Runtime = 'D:\tron1-isaac\isaac-sim-4.5.0',
    [string]$WorkDir = 'D:\tron1-stopping\project',
    [string]$OutputDir = '',
    [string]$Config = '',
    [string]$Suite = '',
    [switch]$Headless,
    [switch]$AutoRun,
    [switch]$ExitAfterTrial,
    [switch]$UiCheck
)
$ErrorActionPreference = 'Stop'
$source = Split-Path $PSScriptRoot -Parent
$python = Join-Path $Runtime 'python.bat'
if (-not (Test-Path $python)) { throw "Isaac Sim runtime missing: $python" }
if (-not (Test-Path (Join-Path $source 'assets\robots\WF_TRON1A\WF_TRON1A.usd'))) {
    throw 'Run python scripts/prepare_assets.py first.'
}
New-Item -ItemType Directory -Force -Path $WorkDir | Out-Null
foreach ($name in @('scripts', 'assets', 'config', 'launch_stopping.cmd')) {
    if ($source -ne $WorkDir) {
        Copy-Item -Path (Join-Path $source $name) -Destination $WorkDir -Recurse -Force
    }
}
if (-not $OutputDir) { $OutputDir = Join-Path 'D:\tron1-stopping\runs' (Get-Date -Format 'yyyyMMdd-HHmmss') }
$simArgs = @((Join-Path $WorkDir 'scripts\stopping_lab.py'), '--output-dir', $OutputDir)
if ($Config) { $simArgs += @('--config', $Config) }
if ($Suite) { $simArgs += @('--suite', $Suite) }
if ($Headless) { $simArgs += '--headless' }
if ($AutoRun) { $simArgs += '--auto-run' }
if ($ExitAfterTrial) { $simArgs += '--exit-after-trial' }
if ($UiCheck) { $simArgs += '--ui-check' }
$env:OMNI_KIT_ACCEPT_EULA = 'YES'
Push-Location $Runtime
try {
    & $python @simArgs
    if ($LASTEXITCODE -ne 0) { throw "Isaac exited with code $LASTEXITCODE" }
    if (($Headless -or $ExitAfterTrial) -and -not (Test-Path (Join-Path $OutputDir 'trial-001\report.json'))) {
        throw "Isaac produced no completed trial report: $OutputDir"
    }
} finally { Pop-Location }
