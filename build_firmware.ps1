[CmdletBinding()]
param(
    [ValidateSet('UART', 'I2C', 'Bootloader', 'All')]
    [string]$Target,

    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',

    [ValidateSet('GCC_ARM', 'ARM', 'IAR')]
    [string]$Toolchain = 'GCC_ARM',

    [ValidateSet('INDEX_FINGER', 'THUMB', 'PINKY')]
    [string]$FingerType = 'INDEX_FINGER',

    [string]$MakePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ApplicationDirectory = $PSScriptRoot
$WorkspaceDirectory = Split-Path -Parent $ApplicationDirectory
$BootloaderDirectory = Join-Path $WorkspaceDirectory 'uart_iic_bootload'

if (-not $MakePath) {
    $makeCommand = Get-Command 'make.exe' -CommandType Application -ErrorAction SilentlyContinue
    if ($makeCommand) {
        $MakePath = $makeCommand.Source
    }
    else {
        $modusToolboxDirectory = Join-Path $env:USERPROFILE 'ModusToolbox'
        $MakePath = Get-ChildItem -Path $modusToolboxDirectory -Directory -Filter 'tools_*' |
            Sort-Object Name -Descending |
            ForEach-Object { Join-Path $_.FullName 'modus-shell\bin\make.exe' } |
            Where-Object { Test-Path -LiteralPath $_ } |
            Select-Object -First 1
    }
}

if (-not $MakePath -or -not (Test-Path -LiteralPath $MakePath -PathType Leaf)) {
    throw '未找到 ModusToolbox make.exe。请使用 -MakePath 指定完整路径。'
}

$MakePath = (Resolve-Path -LiteralPath $MakePath).Path
$modusShellBin = Split-Path -Parent $MakePath
$env:PATH = "$modusShellBin;$env:PATH"
Write-Host "使用 make: $MakePath" -ForegroundColor DarkGray

function Invoke-ProjectBuild {
    param(
        [Parameter(Mandatory)]
        [string]$ProjectDirectory,

        [Parameter(Mandatory)]
        [string]$DisplayName,

        [string]$Transport
    )

    $arguments = @(
        'build'
        "TOOLCHAIN=$Toolchain"
        "CONFIG=$Configuration"
    )
    if ($Transport) {
        $arguments += "TRANSPORT_OPT=$Transport"
    }
    if ($ProjectDirectory -eq $ApplicationDirectory) {
        $arguments += "FINGER_TYPE=$FingerType"
    }

    Write-Host "`n=== 构建 $DisplayName ($Configuration / $Toolchain) ===" -ForegroundColor Cyan
    Write-Host "工程目录: $ProjectDirectory"

    Push-Location $ProjectDirectory
    try {
        & $MakePath @arguments
        if ($LASTEXITCODE -ne 0) {
            throw "$DisplayName 构建失败，make 退出码：$LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }

    Write-Host "=== $DisplayName 构建成功 ===" -ForegroundColor Green
}

function Show-ApplicationArtifacts {
    param([Parameter(Mandatory)][string]$Transport)

    $outputDirectory = Join-Path $ApplicationDirectory "build\$FingerType\$Transport"
    Get-ChildItem -Path $outputDirectory -Recurse -File |
        Where-Object Extension -In '.bin', '.cyacd2' |
        Select-Object -ExpandProperty FullName |
        ForEach-Object { Write-Host "产物: $_" -ForegroundColor DarkGreen }
}

function Build-Application {
    param([Parameter(Mandatory)][ValidateSet('UART', 'I2C')][string]$Transport)

    Invoke-ProjectBuild `
        -ProjectDirectory $ApplicationDirectory `
        -DisplayName "$Transport 应用固件" `
        -Transport $Transport
    Show-ApplicationArtifacts -Transport $Transport
}

function Build-Bootloader {
    Invoke-ProjectBuild `
        -ProjectDirectory $BootloaderDirectory `
        -DisplayName 'UART + I2C 统一 Bootloader'

    $outputDirectory = Join-Path $BootloaderDirectory 'build\DUAL'
    Get-ChildItem -Path $outputDirectory -Recurse -Filter 'bootloader.hex' -File |
        Select-Object -ExpandProperty FullName |
        ForEach-Object { Write-Host "产物: $_" -ForegroundColor DarkGreen }
}

if (-not $Target) {
    Write-Host '请选择需要构建的目标：' -ForegroundColor Yellow
    Write-Host '  1. UART 应用固件'
    Write-Host '  2. I2C 应用固件'
    Write-Host '  3. UART + I2C 统一 Bootloader'
    Write-Host '  4. 全部构建'
    Write-Host '  0. 退出'

    $selection = Read-Host '请输入序号'
    $Target = switch ($selection) {
        '1' { 'UART' }
        '2' { 'I2C' }
        '3' { 'Bootloader' }
        '4' { 'All' }
        '0' { return }
        default { throw "无效选择：$selection" }
    }
}

switch ($Target) {
    'UART'       { Build-Application -Transport 'UART' }
    'I2C'        { Build-Application -Transport 'I2C' }
    'Bootloader' { Build-Bootloader }
    'All' {
        Build-Application -Transport 'UART'
        Build-Application -Transport 'I2C'
        Build-Bootloader
    }
}

Write-Host "`n所选目标已全部构建完成。" -ForegroundColor Green
