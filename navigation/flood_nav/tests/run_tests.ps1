param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$floodTestDir = $PSScriptRoot
$floodOutputDir = Join-Path ([System.IO.Path]::GetTempPath()) ('flood-nav-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $floodOutputDir | Out-Null
$floodExe = Join-Path $floodOutputDir 'test_flood_nav.exe'
$floodSource = Join-Path $floodTestDir '..\flood_nav.c'
$floodInclude = Join-Path $floodTestDir '..'
$floodTest = Join-Path $floodTestDir 'test_flood_nav.c'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-Wpedantic' '-Wconversion' '-Wshadow' '-O2' "-I$floodInclude" $floodSource $floodTest '-o' $floodExe
if ($LASTEXITCODE -ne 0) { throw 'Independent module compilation failed.' }
$floodStdout = Join-Path $floodOutputDir 'stdout.txt'
$floodStderr = Join-Path $floodOutputDir 'stderr.txt'
$floodProcess = Start-Process -FilePath $floodExe -PassThru -WindowStyle Hidden -RedirectStandardOutput $floodStdout -RedirectStandardError $floodStderr
if (-not $floodProcess.WaitForExit(20000)) {
    $floodProcess.Kill()
    throw 'Tests exceeded 20 seconds: possible hang.'
}
$floodProcess.WaitForExit()
Get-Content -LiteralPath $floodStdout
Get-Content -LiteralPath $floodStderr
if ($floodProcess.ExitCode -ne 0) { throw "Tests failed: exit $($floodProcess.ExitCode)" }
Write-Output "Artifacts: $floodOutputDir"
