# bt_run.ps1
#
# One command = zero the pose, then drive to a target, over Bluetooth only.
#
# WHY THIS EXISTS
# ---------------
# The pose is absolute since boot, so a run has to start with 'Z' (zero) or the
# car drives towards a coordinate that no longer means what the operator thinks.
# On demo day that "first send Z" step is exactly the kind of thing that gets
# skipped while someone is talking, and the failure is confusing rather than
# obvious (the car drives off in a plausible direction).
#
#   tools\bt_run.cmd "0 1400"      zero, then drive to (0, 1400) mm
#   tools\bt_run.cmd "-700 700"    negative targets: keep the quotes
#
# Both commands go out on the same port handle, Z first with a short settle, so
# ordering cannot be lost between two processes.
#
# LINK RELIABILITY (added 2026-11-19)
# -----------------------------------
# Opening COM10 can succeed while the radio link is actually down, and then the
# car silently never moves.  bt_common.ps1 reads the firmware's continuous
# telemetry stream to prove the link carries bytes, and retries the open; this
# script refuses to pretend a dead link is a successful run.
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Target = '',
    [string]$Port = 'COM10',
    [int]$SettleMs = 700
)

$ErrorActionPreference = 'Stop'

. "$PSScriptRoot\bt_common.ps1"

if ($Target -eq '')
{
    Write-Host 'usage: bt_run.cmd "x y"     e.g. bt_run.cmd "0 1400"' -ForegroundColor Yellow
    Write-Host '       zeroes the pose, then drives to (x, y) over Bluetooth only' -ForegroundColor DarkGray
    exit 2
}

# Accept "x y" or "x,y"; the firmware wants spaces.
$t = $Target.Trim() -replace ',', ' '
$parts = $t -split '\s+' | Where-Object { $_ -ne '' }
if ($parts.Count -ne 2) { throw ('target must be "x y", got: ' + $Target) }
$x = 0
$y = 0
if (-not [int]::TryParse($parts[0], [ref]$x) -or -not [int]::TryParse($parts[1], [ref]$y))
{
    throw ('target must be two integers, got: ' + $Target)
}

$dist = [Math]::Sqrt([double]($x * $x + $y * $y))
# Straight-line time at PATH_V_MAX_MM_S is a LOWER bound: the axis-aligned path
# drives |y| + |x| and turns in place at the corner, so the real run is longer.
$eta = $dist / 120.0
$legs = [Math]::Abs($y) + [Math]::Abs($x)
$etaLegs = $legs / 120.0

$sp = Open-BtPort -Port $Port
if ($null -eq $sp)
{
    exit 2
}

$ok = $false
try
{
    # Prove the link carries bytes before spending the operator's trust on it.
    $linkOk = Write-BtLinkReport -Port $sp -Ms 500
    if (-not $linkOk)
    {
        Write-Host ''
        Write-Host 'NOT sending the drive command on a dead link.' -ForegroundColor Red
        Write-Host 'Double-click again in a few seconds; if it stays dead, check the module LED.' -ForegroundColor Yellow
        exit 3
    }

    # The first write after opening an SPP port is sometimes swallowed (the same
    # thing happens on the ST-Link VCP, which is why bench_p0 retries its 'h').
    # Losing the Z is the worst possible loss: G then runs against last time's
    # pose, the heading no longer matches the car, and the controller rotates on
    # the spot before doing anything else.  Zeroing twice is harmless, so send it
    # twice rather than hope.
    $sp.Write("Z`n")
    Start-Sleep -Milliseconds 250
    $sp.Write("Z`n")
    Write-Host 'sent: Z (twice - pose + path zeroed)' -ForegroundColor Green
    Start-Sleep -Milliseconds $SettleMs

    $sp.Write(('G {0} {1}' -f $x, $y) + "`n")
    Write-Host ('sent: G {0} {1}' -f $x, $y) -ForegroundColor Green
    Write-Host ('      target is {0:F0} mm away; the path is axis-aligned:' -f $dist) -ForegroundColor DarkGray
    Write-Host ('      straight {0} mm along +y, then one 90 deg turn, then {1} mm along x' -f [Math]::Abs($y), [Math]::Abs($x)) -ForegroundColor DarkGray
    Write-Host ('      expect about {0:F0} s of driving (total path {1:F0} mm)' -f $etaLegs, $legs) -ForegroundColor DarkGray
    Write-Host '      the car stops by itself when the path is done - do not touch it' -ForegroundColor DarkGray

    Start-Sleep -Milliseconds 400
    $ok = $true
}
finally
{
    try { $sp.Close(); $sp.Dispose() } catch { }
}

if (-not $ok) { exit 2 }
exit 0
