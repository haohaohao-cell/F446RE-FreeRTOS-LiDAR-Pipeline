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
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Target = '',
    [string]$Port = 'COM10',
    [int]$SettleMs = 700
)

$ErrorActionPreference = 'Stop'

if ($Target -eq '') {
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
if (-not [int]::TryParse($parts[0], [ref]$x) -or -not [int]::TryParse($parts[1], [ref]$y)) {
    throw ('target must be two integers, got: ' + $Target)
}

$dist = [Math]::Sqrt([double]($x * $x + $y * $y))
$eta = $dist / 120.0          # PATH_V_MAX_MM_S, straight-line only

try {
    $sp = New-Object System.IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
    $sp.ReadTimeout = 200
    $sp.Open()
} catch {
    Write-Host ("cannot open {0}: {1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    Write-Host '  - HC-04 powered and paired?'
    Write-Host '  - is the PC point-cloud viewer holding the port?'
    exit 2
}

try {
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
    Write-Host ('      distance {0:F0} mm, expect about {1:F0} s of driving' -f $dist, $eta) -ForegroundColor DarkGray
    Write-Host '      the car stops by itself when the path is done - do not touch it' -ForegroundColor DarkGray

    Start-Sleep -Milliseconds 400
} finally {
    try { $sp.Close(); $sp.Dispose() } catch { }
}

exit 0
