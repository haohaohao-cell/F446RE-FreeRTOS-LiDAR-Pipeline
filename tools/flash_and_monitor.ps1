# flash_and_monitor.ps1
#
# Flash the Block 4.5 firmware over SWD and capture the USART2 debug output.
#
# Phases:
#   1. Check that the ST-Link can actually see the target (target voltage!).
#   2. Optionally program Debug/CodingDay_F446RE.elf, verify, reset and run.
#   3. Open the ST-Link Virtual COM port and dump whatever the firmware prints,
#      saving a copy under tools/logs/.
#
# The firmware prints, at 115200 8N1 on USART2:
#     <startup banner>
#     #0042 p204 ok198 nr3 iv3 n171 d[210..7900] lat42 p31 S 12 34 255 ...   (10 Hz)
#     [1s] blk 45/1234 q1 1/8 q2 0/4 idle 0 latmax 88 parsemax 41 | ...      (1 Hz)
#
# Usage (prefer the .cmd wrapper, it handles the execution policy):
#     flash_and_monitor.cmd -Probe              only check the SWD connection
#     flash_and_monitor.cmd                     program + capture 15 s
#     flash_and_monitor.cmd -NoFlash            capture only (no programming)
#     flash_and_monitor.cmd -Seconds 30         capture longer
#
# NOTE: header uses plain "#" comments on purpose - Windows PowerShell 5.1
# mis-parses the angle-bracket block-comment form in some positions.
[CmdletBinding()]
param(
    [string]$Elf      = "",
    [string]$Port     = "COM8",
    [string]$Baud     = "115200",
    [string]$Seconds  = "15",
    [string]$Cli      = "",
    [switch]$Probe,
    [switch]$NoFlash
)

$ErrorActionPreference = "Stop"

# $PSScriptRoot is not populated while param() defaults are evaluated under
# "powershell -File", so resolve the default path here instead.
if ([string]::IsNullOrWhiteSpace($Elf)) {
    $Elf = Join-Path $PSScriptRoot "..\Debug\CodingDay_F446RE.elf"
}

# --- locate STM32_Programmer_CLI.exe ---------------------------------------
# Every machine installs STM32CubeIDE somewhere else, so search the usual roots
# instead of hard-coding one path.  Override with -Cli if it still fails.
if ([string]::IsNullOrWhiteSpace($Cli)) {
    $roots = @(
        "$env:ProgramFiles\STMicroelectronics",
        "${env:ProgramFiles(x86)}\STMicroelectronics",
        "$env:LOCALAPPDATA\Programs\STM32CubeIDE",
        "C:\ST",
        "D:\CubeIDE",
        "C:\CubeIDE",
        "E:\CubeIDE"
    ) | Where-Object { $_ -and (Test-Path $_) }

    foreach ($root in $roots) {
        $found = Get-ChildItem $root -Recurse -Filter "STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue |
                 Select-Object -First 1
        if ($found) { $Cli = $found.FullName; break }
    }
}
if ([string]::IsNullOrWhiteSpace($Cli) -or -not (Test-Path $Cli)) {
    throw "STM32_Programmer_CLI.exe not found. Install STM32CubeIDE / STM32CubeProgrammer, or pass -Cli <full path>."
}

$Baud = [int]$Baud
$Seconds = [int]$Seconds
if ($Seconds -le 0) { $Seconds = 15 }

$Elf = [System.IO.Path]::GetFullPath($Elf)
if (-not (Test-Path $Elf)) { throw "ELF not found: $Elf  (build the project first)" }

function Get-SwdStatus {
    # mode=UR -> connect under reset (works when the target is stuck in a loop)
    $raw = & $Cli -c port=SWD mode=UR freq=4000 -q 2>&1 | Out-String
    $volt = -1.0
    if ($raw -match 'Voltage\s*:\s*([\d.]+)\s*V') { $volt = [double]$Matches[1] }
    New-Object PSObject -Property @{
        Ok           = (($raw -match 'Device name\s*:\s*STM32') -and ($raw -notmatch 'No STM32 target found'))
        ProbeMissing = ($raw -match 'No debug probe detected')
        Voltage      = $volt
        Raw          = $raw
    }
}

function Show-ProbeHint($status) {
    Write-Host ""
    Write-Host "==============================================================" -ForegroundColor Red
    Write-Host " No debug probe detected - the ST-Link is not connected." -ForegroundColor Red
    Write-Host "==============================================================" -ForegroundColor Red
    Write-Host ""
    Write-Host "Check, in this order:" -ForegroundColor Yellow
    Write-Host "  1. Is the ST-Link USB cable plugged into THIS computer?"
    Write-Host "  2. Is it a data cable, not a charge-only one?"
    Write-Host "  3. Did the ST-Link enumerate?  Device Manager -> Ports should"
    Write-Host "     list 'STMicroelectronics STLink Virtual COM Port (COMx)'."
    Write-Host "  4. Close STM32CubeIDE's debug session if one is holding the probe."
    Write-Host ""
    Write-Host "Raw output:" -ForegroundColor DarkGray
    Write-Host $status.Raw -ForegroundColor DarkGray
}

function Show-PowerHint($status) {
    Write-Host ""
    Write-Host "==============================================================" -ForegroundColor Red
    Write-Host (" ST-Link sees NO target. Target voltage = {0} V" -f $status.Voltage) -ForegroundColor Red
    Write-Host "==============================================================" -ForegroundColor Red
    Write-Host ""
    Write-Host "Check, in this order:" -ForegroundColor Yellow
    Write-Host "  1. Is the MCU board powered at all?  The USB cable normally feeds"
    Write-Host "     only the ST-Link.  Turn the car's main power switch ON (or"
    Write-Host "     supply 5 V / battery to the board's power input)."
    Write-Host "  2. Stand-alone ST-Link dongle: wire pin 1 (VREF / T_VCC) to the"
    Write-Host "     board's 3V3.  The V3 firmware will not open SWD when it cannot"
    Write-Host "     sense the target voltage."
    Write-Host "  3. ST-Link pin 4 (GND) must be connected to the board GND."
    Write-Host "  4. Board with on-board ST-Link: check the power-source jumper and"
    Write-Host "     measure 3V3 with a multimeter."
    Write-Host "  5. SWDIO (pin 2) / SWCLK (pin 4) not swapped; NRST (pin 5) optional."
    Write-Host ""
    Write-Host "Raw output:" -ForegroundColor DarkGray
    Write-Host $status.Raw -ForegroundColor DarkGray
}

# --- 1. probe ---------------------------------------------------------------
$swd = Get-SwdStatus
if (-not $swd.Ok) {
    if ($swd.ProbeMissing) { Show-ProbeHint $swd } else { Show-PowerHint $swd }
    exit 2
}

Write-Host ("Target OK (voltage {0} V)" -f $swd.Voltage) -ForegroundColor Green
$swd.Raw -split "`n" | Where-Object { $_ -match 'Device|Flash|Voltage|ST-LINK' } |
    ForEach-Object { Write-Host ("   " + $_.Trim()) -ForegroundColor DarkGray }

if ($Probe) { exit 0 }

# --- 2. open the serial port FIRST ------------------------------------------
# The firmware prints its banner within a few hundred ms of reset, so the port
# has to be open before the target is reset or the banner is lost.
$logDir = Join-Path $PSScriptRoot "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$logPath = Join-Path $logDir (("serial_{0:yyyyMMdd_HHmmss}.txt" -f (Get-Date)))

$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, "None", 8, "One")
$sp.ReadTimeout = 300
try { $sp.Open() } catch { throw ("Cannot open {0}: {1}" -f $Port, $_.Exception.Message) }
$sp.DiscardInBuffer()
$sb = New-Object System.Text.StringBuilder

# --- 3. flash ---------------------------------------------------------------
if (-not $NoFlash) {
    Write-Host ""
    Write-Host "Programming $Elf ..." -ForegroundColor Cyan
    $flashOut = & $Cli -c port=SWD mode=UR freq=4000 -d $Elf -v -rst 2>&1 | Out-String
    if ($flashOut -match 'Error|No STM32 target found') {
        $sp.Close(); $sp.Dispose()
        Write-Host $flashOut -ForegroundColor Red
        throw "Programming failed."
    }
    $flashOut -split "`n" |
        Where-Object { $_ -match 'Erasing|Download|Verify|reset|Reset|File download' } |
        ForEach-Object { Write-Host ("   " + $_.Trim()) -ForegroundColor DarkGray }
}

# --- 4. capture serial ------------------------------------------------------
Write-Host ""
Write-Host ("Listening on {0} @ {1} 8N1 for {2} s  (Ctrl+C to stop early)" -f $Port, $Baud, $Seconds) -ForegroundColor Cyan

$deadline = (Get-Date).AddSeconds($Seconds)
try {
    while ((Get-Date) -lt $deadline) {
        $chunk = ""
        try { $chunk = $sp.ReadExisting() } catch { $chunk = "" }
        if ($chunk) {
            [void]$sb.Append($chunk)
            Write-Host -NoNewline $chunk
        } else {
            Start-Sleep -Milliseconds 40
        }
    }
} finally {
    $sp.Close()
    $sp.Dispose()
}

$text = $sb.ToString()
[System.IO.File]::WriteAllText($logPath, $text, (New-Object System.Text.UTF8Encoding($false)))

Write-Host ""
Write-Host ""
Write-Host ("Captured {0} characters -> {1}" -f $text.Length, $logPath) -ForegroundColor Green

# --- 4. quick verdict -------------------------------------------------------
Write-Host ""
Write-Host "--- verdict -------------------------------------------------" -ForegroundColor Yellow
if ($text.Length -eq 0) {
    Write-Host "  NOTHING received." -ForegroundColor Red
    Write-Host ("  - wrong COM port? (expect the ST-Link VCP, currently {0})" -f $Port)
    Write-Host "  - is the firmware running? pause with the debugger:"
    Write-Host "    a PC parked in Error_Handler / vApplicationMallocFailedHook /"
    Write-Host "    vApplicationStackOverflowHook / vPortValidateInterruptPriority"
    Write-Host "    prints nothing and never returns."
    exit 3
}

$statsLine = ($text -split "`r?`n" | Where-Object { $_ -match '^\[1s\]' } | Select-Object -Last 1)
if ($statsLine) {
    Write-Host "  last 1 Hz line:" -ForegroundColor Green
    Write-Host ("    " + $statsLine)
    $m = [regex]::Match($statsLine, 'blk\s+(\d+)/(\d+).*?q1\s+(\d+)/(\d+).*?idle\s+(\d+)')
    if ($m.Success) {
        $rate = [int]$m.Groups[1].Value
        $q1   = [int]$m.Groups[3].Value
        $q1max= [int]$m.Groups[4].Value
        $idle = [int]$m.Groups[5].Value
        if (($rate -eq 0) -and ($idle -gt 0)) {
            Write-Host "  blk=0 and idle increasing -> LiDAR is silent (no blocks at all)." -ForegroundColor Red
            Write-Host "    check USART3 wiring / 460800 baud / RPLidar power / SCAN command."
        } elseif ($q1 -ge $q1max) {
            Write-Host "  Queue 1 saturated -> parser cannot keep up with the DMA rate." -ForegroundColor Red
        } else {
            Write-Host ("  Pipeline looks healthy: ~{0} blocks/s, q1 peak {1}, idle {2}" -f $rate, $q1, $idle) -ForegroundColor Green
        }
    }
} else {
    Write-Host "  Got data but no [1s] statistics line - check the 1 Hz code path."
}

$blockLine = ($text -split "`r?`n" | Where-Object { $_ -match '^#' } | Select-Object -Last 1)
if ($blockLine) {
    Write-Host "   last 10 Hz line:"
    Write-Host ("    " + $blockLine)
}
