param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$guardOutputDir = Join-Path ([System.IO.Path]::GetTempPath()) ('lidar-guard-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $guardOutputDir | Out-Null
$guardExe = Join-Path $guardOutputDir 'test_lidar_guard.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-Wpedantic' '-Wconversion' '-Wshadow' '-O2' "-I$PSScriptRoot\.." "$PSScriptRoot\..\lidar_guard.c" "$PSScriptRoot\test_lidar_guard.c" '-o' $guardExe '-lm'
if ($LASTEXITCODE -ne 0) { throw 'Independent collision module compilation failed.' }
$guardStdout = Join-Path $guardOutputDir 'stdout.txt'
$guardStderr = Join-Path $guardOutputDir 'stderr.txt'
$guardProcess = Start-Process -FilePath $guardExe -PassThru -WindowStyle Hidden -RedirectStandardOutput $guardStdout -RedirectStandardError $guardStderr
if (-not $guardProcess.WaitForExit(20000)) {
    $guardProcess.Kill()
    throw 'Tests exceeded 20 seconds: possible hang.'
}
$guardProcess.WaitForExit()
Get-Content -LiteralPath $guardStdout
Get-Content -LiteralPath $guardStderr
if ($guardProcess.ExitCode -ne 0) { throw "Tests failed: exit $($guardProcess.ExitCode)" }
Write-Output "Artifacts: $guardOutputDir"
