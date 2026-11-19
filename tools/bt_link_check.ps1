# bt_link_check.ps1
#
# Prove the Bluetooth SPP link is alive BEFORE the car is placed on the tape.
#
# WHY THIS EXISTS
# ---------------
# The HC-04's LED blinks when the radio link is down, but by the time anyone
# notices that on stage the car has already been commanded and has not moved.
# This script answers the question objectively in three seconds: the firmware
# streams a ~256 byte telemetry frame 20 times a second over UART5, so a healthy
# link delivers roughly 5000 bytes per second.  0 bytes means the link is dead
# no matter what the port open() returned, and a few hundred means it is fading.
#
# Usage:
#     tools\bt_link_check.cmd                3 s sample, one shot
#     tools\bt_link_check.cmd -Sec 10        sample longer
#     tools\bt_link_check.cmd -Loop 30       watch it for 30 s (drops show up as gaps)
#
# Exit code: 0 = link healthy, 1 = weak, 2 = no data / port not available.
[CmdletBinding()]
param(
    [string]$Port = 'COM10',
    [int]$Sec = 3,
    [int]$Loop = 0,
    [int]$Attempts = 6
)

$ErrorActionPreference = 'Stop'

. "$PSScriptRoot\bt_common.ps1"

Write-Host ("Bluetooth link check on {0}" -f $Port) -ForegroundColor Cyan
Write-Host 'the car must be powered on: the firmware is the thing sending the bytes.' -ForegroundColor DarkGray

$sp = Open-BtPort -Port $Port -Attempts $Attempts
if ($null -eq $sp)
{
    exit 2
}

$worst = 'ok'
try
{
    $rounds = 1
    if ($Loop -gt 0) { $rounds = [Math]::Max(1, [int]($Loop / [Math]::Max(1, $Sec))) }

    for ($r = 1; $r -le $rounds; $r++)
    {
        $m = Measure-BtLink -Port $sp -Ms ($Sec * 1000)
        $verdict = 'ok'
        if ($m.Bytes -eq 0) { $verdict = 'dead' }
        elseif ($m.Rate -lt 1000) { $verdict = 'weak' }

        $colour = 'Green'
        if ($verdict -eq 'dead') { $colour = 'Red'; $worst = 'dead' }
        elseif ($verdict -eq 'weak') { $colour = 'Yellow'; if ($worst -ne 'dead') { $worst = 'weak' } }

        Write-Host ("[{0}/{1}] {2,7} B in {3,5} ms  ->  {4,6} B/s  {5}" -f `
            $r, $rounds, $m.Bytes, $m.Ms, $m.Rate, $verdict.ToUpper()) -ForegroundColor $colour

        if ($verdict -eq 'ok')
        {
            Write-Host '  healthy: the radio link, the module UART and the PC adapter all work.' -ForegroundColor DarkGray
        }
        elseif ($verdict -eq 'weak')
        {
            Write-Host '  the link is up but losing data. Expect a missed command.' -ForegroundColor Yellow
        }
        else
        {
            Write-Host '  no bytes at all: the car will not receive anything.' -ForegroundColor Red
            Write-Host '  - is the car powered on (battery connected, switch on)?' -ForegroundColor Yellow
            Write-Host '  - module LED steady or blinking?' -ForegroundColor Yellow
            Write-Host '  - is the point-cloud viewer open and holding COM10?' -ForegroundColor Yellow
            Write-Host '  - Windows: Device Manager > Bluetooth adapter > Power Management' -ForegroundColor DarkGray
        }
    }
}
finally
{
    try { $sp.Close(); $sp.Dispose() } catch { }
}

switch ($worst)
{
    'ok'   { exit 0 }
    'weak' { exit 1 }
    default { exit 2 }
}
