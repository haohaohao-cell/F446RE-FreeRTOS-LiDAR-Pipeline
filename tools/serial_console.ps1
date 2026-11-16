# serial_console.ps1
#
# Live view of the STM32 debug console (USART2 -> ST-Link VCP).
#
# The firmware prints two kinds of line. Both of them live HERE, not in the
# point-cloud window (that one is the binary telemetry on UART5/Bluetooth):
#
#   #0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S ...   <- one DMA block
#   [1s] blk 25/472 q1 0/8 q2 1/4 idle 5 latmax 22 ...             <- once per second
#
# Everything you need for the demo (blk, idle, q1, q2, latmax, parsemax, heap,
# stk, hal/k) is in the [1s] line, which is highlighted in cyan.
#
# Usage:
#     tools\serial_console.cmd                  COM8 @ 115200, run until Ctrl+C
#     tools\serial_console.cmd -Port COM4       another port
#     tools\serial_console.cmd -Seconds 5       run for 5 s then stop (self-test)
[CmdletBinding()]
param(
    [string]$Port    = "COM8",
    [int]   $Baud    = 115200,
    [int]   $Seconds = 0
)

$ErrorActionPreference = "Stop"

$logDir = Join-Path $PSScriptRoot "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$logPath = Join-Path $logDir (("console_{0:yyyyMMdd_HHmmss}.txt" -f (Get-Date)))

Write-Host ""
Write-Host ("CodingDay debug console  |  {0} @ {1} 8N1" -f $Port, $Baud) -ForegroundColor White
Write-Host "  [1s] = statistics (blk / q1 / idle / latmax / heap / stk)   [cyan]" -ForegroundColor DarkGray
Write-Host "  #    = one DMA block  (p / ok / nr / iv / n / d / lat / p)  [grey]" -ForegroundColor DarkGray
Write-Host "  [cmd] = console command echoed by the firmware             [magenta]" -ForegroundColor DarkGray
Write-Host ("  log -> {0}" -f $logPath) -ForegroundColor DarkGray
Write-Host "  TYPE:  s = stop the LiDAR   g = restart it   h = help" -ForegroundColor DarkYellow
Write-Host "  Ctrl+C to stop" -ForegroundColor DarkGray
Write-Host ""

$interactive = -not [Console]::IsInputRedirected

$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, "None", 8, "One")
$sp.ReadTimeout = 200
$sp.Open()
$sp.DiscardInBuffer()

$log = New-Object System.IO.StreamWriter($logPath, $false, (New-Object System.Text.UTF8Encoding($false)))
$log.AutoFlush = $true

$buf = ""
$deadline = if ($Seconds -gt 0) { (Get-Date).AddSeconds($Seconds) } else { $null }

try {
    while ($true) {
        if ($deadline -and ((Get-Date) -gt $deadline)) { break }

        $chunk = ""
        try { $chunk = $sp.ReadExisting() } catch { $chunk = "" }
        if ($chunk) { $buf += $chunk }

        while (($idx = $buf.IndexOf("`n")) -ge 0) {
            $line = $buf.Substring(0, $idx).TrimEnd("`r")
            $buf = $buf.Substring($idx + 1)
            if ($line.Length -eq 0) { continue }

            $log.WriteLine($line)

            if ($line -match '^\[1s\]') {
                Write-Host $line -ForegroundColor Cyan
            }
            elseif ($line -match '^\[cmd\]') {
                Write-Host $line -ForegroundColor Magenta
            }
            elseif ($line -match '^#') {
                Write-Host $line -ForegroundColor DarkGray
            }
            elseif ($line -match 'timebase|ready|Block4\.5') {
                Write-Host $line -ForegroundColor Yellow
            }
            else {
                Write-Host $line
            }
        }

        # forward a typed character to the MCU: s = stop LiDAR, g = restart
        if ($interactive) {
            try {
                if ([Console]::KeyAvailable) {
                    $key = [Console]::ReadKey($true)
                    if ($key.Key -eq [ConsoleKey]::Escape) { break }
                    $ch = [string]$key.KeyChar
                    if ($ch.Length -gt 0) {
                        $sp.Write($ch)
                        Write-Host ("        >> sent '{0}'" -f $ch) -ForegroundColor DarkYellow
                    }
                }
            } catch { $interactive = $false }
        }

        if (-not $chunk) { Start-Sleep -Milliseconds 20 }
    }
}
finally {
    try { $sp.Close(); $sp.Dispose() } catch { }
    $log.Close()
    Write-Host ""
    Write-Host ("stopped - log saved to {0}" -f $logPath) -ForegroundColor DarkGray
}
