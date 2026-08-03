param(
    [string]$GodotExe = ""
)

$ErrorActionPreference = "Stop"
$expectedVersion = "4.7.1.stable.official.a13da4feb"
$candidates = @(
    $GodotExe,
    $env:CATHSIM_GODOT_EXE,
    "D:\Program Files\Godot\Godot_v4.7.1-stable_win64_console.exe",
    "D:\Program Files\Godot\Godot_v4.7.1-stable_win64.exe"
)
$resolved = $null
foreach ($candidate in $candidates | Where-Object { $_ } | Select-Object -Unique) {
    if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        $version = (& $candidate --version 2>&1 | Out-String).Trim()
        if ($LASTEXITCODE -eq 0 -and $version -eq $expectedVersion) {
            $resolved = (Resolve-Path -LiteralPath $candidate).Path
            break
        }
    }
}
if (-not $resolved) {
    throw "CathSim requires Godot $expectedVersion. Set CATHSIM_GODOT_EXE or pass -GodotExe."
}
$forwarded = @($args)
if ($forwarded.Count -eq 0) {
    & $resolved --version
} else {
    & $resolved @forwarded
}
exit $LASTEXITCODE
