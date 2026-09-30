param(
    [string]$ImagePath = "",
    [string]$Language = "zh-Hans-CN",
    [switch]$Server
)

$ErrorActionPreference = "Stop"
[Console]::InputEncoding = [System.Text.UTF8Encoding]::new($false)
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

Add-Type -AssemblyName System.Runtime.WindowsRuntime
$null = [Windows.Storage.StorageFile, Windows.Storage, ContentType = WindowsRuntime]
$null = [Windows.Storage.FileAccessMode, Windows.Storage, ContentType = WindowsRuntime]
$null = [Windows.Storage.Streams.IRandomAccessStream, Windows.Storage.Streams, ContentType = WindowsRuntime]
$null = [Windows.Graphics.Imaging.BitmapDecoder, Windows.Graphics.Imaging, ContentType = WindowsRuntime]
$null = [Windows.Graphics.Imaging.SoftwareBitmap, Windows.Graphics.Imaging, ContentType = WindowsRuntime]
$null = [Windows.Media.Ocr.OcrEngine, Windows.Foundation, ContentType = WindowsRuntime]
$null = [Windows.Media.Ocr.OcrResult, Windows.Foundation, ContentType = WindowsRuntime]
$null = [Windows.Globalization.Language, Windows.Globalization, ContentType = WindowsRuntime]

function Wait-WindowsRuntimeOperation {
    param(
        [Parameter(Mandatory = $true)]
        $Operation,

        [Parameter(Mandatory = $true)]
        [Type]$ResultType
    )

    $method = [System.WindowsRuntimeSystemExtensions].GetMethods() |
        Where-Object {
            $_.Name -eq "AsTask" -and
            $_.IsGenericMethod -and
            $_.GetParameters().Count -eq 1
        } |
        Select-Object -First 1
    $task = $method.MakeGenericMethod($ResultType).Invoke($null, @($Operation))
    $task.Wait()
    return $task.Result
}

function Invoke-WindowsOcr {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        $Engine
    )

    $resolvedPath = (Resolve-Path -LiteralPath $Path).Path
    $stream = $null
    try {
        $file = Wait-WindowsRuntimeOperation `
            ([Windows.Storage.StorageFile]::GetFileFromPathAsync($resolvedPath)) `
            ([Windows.Storage.StorageFile])
        $stream = Wait-WindowsRuntimeOperation `
            ($file.OpenAsync([Windows.Storage.FileAccessMode]::Read)) `
            ([Windows.Storage.Streams.IRandomAccessStream])
        $decoder = Wait-WindowsRuntimeOperation `
            ([Windows.Graphics.Imaging.BitmapDecoder]::CreateAsync($stream)) `
            ([Windows.Graphics.Imaging.BitmapDecoder])
        $bitmap = Wait-WindowsRuntimeOperation `
            ($decoder.GetSoftwareBitmapAsync()) `
            ([Windows.Graphics.Imaging.SoftwareBitmap])
        $result = Wait-WindowsRuntimeOperation `
            ($Engine.RecognizeAsync($bitmap)) `
            ([Windows.Media.Ocr.OcrResult])
        return $result.Text
    }
    finally {
        if ($null -ne $stream) {
            $stream.Dispose()
        }
    }
}

function ConvertTo-Base64Utf8 {
    param([AllowEmptyString()][string]$Text)
    return [Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($Text))
}

$recognizerLanguage = [Windows.Globalization.Language]::new($Language)
$engine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromLanguage($recognizerLanguage)
if ($null -eq $engine) {
    $available = [Windows.Media.Ocr.OcrEngine]::AvailableRecognizerLanguages |
        ForEach-Object { $_.LanguageTag }
    throw "Windows未安装OCR语言包 $Language；可用语言：$($available -join ', ')"
}

if ($Server) {
    [Console]::Out.WriteLine("READY`t$($engine.RecognizerLanguage.LanguageTag)")
    [Console]::Out.Flush()
    while ($null -ne ($request = [Console]::In.ReadLine())) {
        try {
            $text = Invoke-WindowsOcr -Path $request -Engine $engine
            [Console]::Out.WriteLine("OK`t$(ConvertTo-Base64Utf8 $text)")
        }
        catch {
            [Console]::Out.WriteLine("ERROR`t$(ConvertTo-Base64Utf8 $_.Exception.Message)")
        }
        [Console]::Out.Flush()
    }
    exit 0
}

if ([string]::IsNullOrWhiteSpace($ImagePath)) {
    throw "非Server模式必须提供ImagePath"
}
[Console]::Write((Invoke-WindowsOcr -Path $ImagePath -Engine $engine))
