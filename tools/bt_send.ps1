# bt_send.ps1
#
# Send one navigation command over the Bluetooth SPP link and nothing else.
#
# WHY THIS EXISTS
# ---------------
# nav_run.ps1 needs BOTH links: it sends "G x y" over Bluetooth but reads the
# resulting pose back over the ST-Link USB serial (COM8), because the pose is
# only printed on USART2 - the 10 Hz telemetry frame carries the IMU and the
# lidar scan, not x/y.
#
# That is fine on the bench but wrong for the acceptance run: a USB cable
# trailing 1.4 m behind a 200 g car drags it off line, and a "Bluetooth
# navigation" demo that is tethered looks exactly like what it is.  For the
# acceptance the car runs untethered, this script sends the command, and the
# endpoint is measured with a tape - which is the number the marking scheme
# asks for anyway.
#
# Usage (or use bt_send.cmd):
#     tools\bt_send.ps1 "G 0 1400"      drive to (0, 1400) mm
#     tools\bt_send.ps1 "S"             stop and clear the path
#     tools\bt_send.ps1 "P"             query (no reply yet - see NEXT_SESSION 8)
[CmdletBinding()]
param(
    [Parameter(Position = 0, Mandatory = $true)]
    [string]$Command,
    [string]$Port = 'COM10'
)

$ErrorActionPreference = 'Stop'

# The firmware splits on '\n' (bt_rx.c), and bt_protocol.c tolerates a trailing
# '\r'.  One line, one command: sending two commands back to back is how the
# late-STOP bug (HANDOVER 12.2 pit E) was born.
$line = $Command.Trim()
if ($line.Length -eq 0) { throw 'empty command' }

try {
    $sp = New-Object System.IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
    $sp.ReadTimeout = 200
    $sp.Open()
} catch {
    Write-Host ("cannot open {0}: {1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    Write-Host '  - is the HC-04 powered and paired?  COM10 was the SPP port on 2026-11-17'
    Write-Host '  - is the PC point-cloud viewer (pc_app) holding the port?'
    exit 2
}

try {
    $sp.Write($line + "`n")
    # Idempotent single-letter commands (S, Z, P) go twice: the first write after
    # opening an SPP port is occasionally swallowed, and a lost S means the car
    # keeps driving.
    if ($line.Length -eq 1) {
        Start-Sleep -Milliseconds 150
        $sp.Write($line + "`n")
    }
    # Give the SPP buffer time to actually drain before the handle closes:
    # closing immediately after Write() is exactly the failure that made a late
    # STOP arrive and wipe a freshly generated path.
    Start-Sleep -Milliseconds 400
    Write-Host ("sent over {0}: {1}" -f $Port, $line) -ForegroundColor Green
} finally {
    try { $sp.Close(); $sp.Dispose() } catch { }
}

exit 0
