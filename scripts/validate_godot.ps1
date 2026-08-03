param(
    [string]$GodotExe = "",
    [string]$ProjectPath = "godot_client",
    [string]$LogPath = ".tmp/godot/headless.log",
    [string]$ExpectedVersion = "4.7.1.stable.official.a13da4feb"
)

$ErrorActionPreference = "Stop"

function Resolve-GodotExe {
    param([string]$ExplicitPath)

    if ($ExplicitPath) {
        if (-not (Test-Path -LiteralPath $ExplicitPath -PathType Leaf)) {
            throw "Godot executable not found: $ExplicitPath"
        }
        return (Resolve-Path -LiteralPath $ExplicitPath).Path
    }

    $fallbacks = @(
        "D:\Program Files\Godot\Godot_v4.7.1-stable_win64_console.exe",
        "D:\Program Files\Godot\Godot_v4.7.1-stable_win64.exe"
    )
    foreach ($folder in @(
        (Join-Path $env:USERPROFILE "Desktop"),
        (Join-Path $env:USERPROFILE "Downloads")
    )) {
        if (Test-Path -LiteralPath $folder -PathType Container) {
            $fallbacks += Get-ChildItem -LiteralPath $folder -Filter "Godot*_console.exe" -File |
                Sort-Object LastWriteTime -Descending |
                Select-Object -ExpandProperty FullName
            $fallbacks += Get-ChildItem -LiteralPath $folder -Filter "Godot*.exe" -File |
                Where-Object Name -NotLike "*_console.exe" |
                Sort-Object LastWriteTime -Descending |
                Select-Object -ExpandProperty FullName
        }
    }
    foreach ($candidate in $fallbacks) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }

    foreach ($command in @("godot", "godot4")) {
        $found = Get-Command $command -ErrorAction SilentlyContinue
        if ($found) {
            return $found.Source
        }
    }

    throw "Godot CLI not found. Add Godot to PATH or pass -GodotExe <path>."
}

$repoRoot = Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")
$projectFullPath = Resolve-Path -LiteralPath (Join-Path $repoRoot $ProjectPath)
$logFullPath = Join-Path $repoRoot $LogPath
$logDir = Split-Path -Parent $logFullPath
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$importLogPath = Join-Path $logDir "import.log"

$godot = Resolve-GodotExe $GodotExe
Write-Host "Godot: $godot"
$detectedVersion = (& $godot --version 2>&1 | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $detectedVersion -ne $ExpectedVersion) {
    throw "Expected Godot $ExpectedVersion, got '$detectedVersion' from $godot"
}
Write-Host "Godot version: $detectedVersion"

Write-Host "Importing Godot assets: $projectFullPath"
& $godot --headless --xr-mode off --path $projectFullPath --log-file $importLogPath --import
if ($LASTEXITCODE -ne 0) {
    throw "Godot asset import failed with exit code $LASTEXITCODE. Log: $importLogPath"
}

Write-Host "Validating Godot project: $projectFullPath"
& $godot --headless --xr-mode off --path $projectFullPath --log-file $logFullPath --editor --quit

if ($LASTEXITCODE -ne 0) {
    throw "Godot validation failed with exit code $LASTEXITCODE. Log: $logFullPath"
}

$errors = Select-String -LiteralPath $logFullPath -Pattern "SCRIPT ERROR:|Parse Error:|Failed loading resource" -Encoding UTF8
if ($errors) {
    throw "Godot logged project/script errors. Log: $logFullPath`n$($errors.Line -join [Environment]::NewLine)"
}

Write-Host "Godot validation passed. Log: $logFullPath"
