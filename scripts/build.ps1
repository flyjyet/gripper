param(
  [string]$Preset = "dev-zig"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$cmake = Join-Path $repoRoot ".venv\Scripts\cmake.exe"

if (-not (Test-Path $cmake)) {
  throw "CMake not found: $cmake"
}

Push-Location $repoRoot
try {
  & $cmake --preset $Preset
  & $cmake --build --preset $Preset
}
finally {
  Pop-Location
}
