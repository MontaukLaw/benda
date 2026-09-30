param(
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvDir = Join-Path $ScriptDir ".venv-build-py312"
$PythonExe = Join-Path $VenvDir "Scripts\python.exe"
$BuildWorkDir = Join-Path $ScriptDir ("build_exe_work_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
$OutputExe = Join-Path $ScriptDir "dist\algo_monitor.exe"

function Write-Step($Message) {
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Get-UvPythonExe {
    $UvPythonRoot = Join-Path $env:USERPROFILE "AppData\Roaming\uv\python"
    if (-not (Test-Path -LiteralPath $UvPythonRoot)) {
        return $null
    }

    return Get-ChildItem -Path $UvPythonRoot -Recurse -Filter python.exe -ErrorAction SilentlyContinue |
        Where-Object {
            $_.FullName -notlike "*\Lib\venv\scripts\*" -and
            $_.FullName -match "cpython-3\.12"
        } |
        Sort-Object FullName |
        Select-Object -First 1 -ExpandProperty FullName
}

function New-BuildVenv($TargetDir) {
    $commands = @()

    if (Get-Command py -ErrorAction SilentlyContinue) {
        $commands += @{ Exe = "py"; Args = @("-3.12", "-m", "venv", $TargetDir) }
    }
    if (Get-Command uv -ErrorAction SilentlyContinue) {
        $commands += @{ Exe = "uv"; Args = @("venv", "--python", "3.12", $TargetDir) }
    }
    $UvPythonExe = Get-UvPythonExe
    if ($UvPythonExe) {
        $commands += @{ Exe = $UvPythonExe; Args = @("-m", "venv", $TargetDir) }
    }

    foreach ($command in $commands) {
        Write-Host "Trying: $($command.Exe) $($command.Args -join ' ')"
        & $command.Exe @($command.Args)
        if (Test-Path -LiteralPath $PythonExe) {
            return
        }
    }

    throw "Failed to create virtual environment: $TargetDir"
}

Write-Step "Preparing build virtual environment"
if (-not (Test-Path -LiteralPath $PythonExe)) {
    New-BuildVenv $VenvDir
}

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Failed to create virtual environment: $VenvDir"
}

if (-not $SkipInstall) {
    Write-Step "Installing build dependencies"
    $PipReady = $false
    try {
        & $PythonExe -c "import pip" 2>$null
        $PipReady = ($LASTEXITCODE -eq 0)
    } catch {
        $PipReady = $false
    }

    if ($PipReady) {
        & $PythonExe -m pip install --upgrade pip
        & $PythonExe -m pip install pyinstaller pyserial pyqtgraph PySide6 python-can opencv-python rapidocr onnxruntime
    } else {
        $BootstrapPython = Get-UvPythonExe
        if (-not $BootstrapPython) {
            throw "The build virtual environment has no pip, and no uv-managed Python with pip was found."
        }
        & $BootstrapPython -m pip --python $PythonExe install pyinstaller pyserial pyqtgraph PySide6 python-can opencv-python rapidocr onnxruntime
    }
}

Write-Step "Building algo_monitor.exe"
Push-Location $ScriptDir
try {
    & $PythonExe -m PyInstaller --noconfirm --clean --onefile --windowed --hidden-import can.interfaces.pcan --collect-all rapidocr --collect-all onnxruntime --add-data "windows_ocr.ps1;." --add-data "force_signal_pressure_maps\force_signal_pressure_map_latest.json;force_signal_pressure_maps" --name algo_monitor --workpath $BuildWorkDir --distpath (Join-Path $ScriptDir "dist") main.py
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

if (-not (Test-Path -LiteralPath $OutputExe)) {
    throw "Build finished but output exe was not found: $OutputExe"
}

Write-Step "Build complete"
Write-Host $OutputExe -ForegroundColor Green
