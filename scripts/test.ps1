param(
  [string]$Preset = "dev-zig"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$cmake = Join-Path $repoRoot ".venv\Scripts\cmake.exe"
$ctest = Join-Path $repoRoot ".venv\Scripts\ctest.exe"

if (-not (Test-Path $cmake)) {
  throw "CMake not found: $cmake"
}
if (-not (Test-Path $ctest)) {
  throw "CTest not found: $ctest"
}

Push-Location $repoRoot
try {
  & $cmake --preset $Preset
  & $cmake --build --preset $Preset
  & $ctest --preset $Preset
}
finally {
  Pop-Location
}
