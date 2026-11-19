# banner.ps1
#
# Show the boot banner without reflashing anything.
#
# WHY THIS EXISTS
# ---------------
# The two lines that matter at the venue are printed ONCE, within a few hundred
# milliseconds of reset, and then never again:
#
#   [drive] frame: REVERSED (drives wheels-first)
#   Block4.5 Part2 ready imu=0 oled=0 i2c=0 bt=0 log=0
#
# The first says whether the reversed-drive build is the one on the chip; the
# second says whether the IMU and the panel came up.  flash_and_monitor.cmd
# shows them too, but only because it reprograms the part - which is a silly
# thing to do just to read two lines.
#
# So: open the COM port FIRST, then reset the target over SWD, then listen.
# No programming, no erase, nothing that can leave the car in a different state
# than it is in now (apart from the reset itself).
[CmdletBinding()]
param(
    [string]$Port    = "COM8",
    [int]$Seconds    = 6,
    [string]$Cli     = ""
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($Cli)) {
    $roots = @(
        "$env:ProgramFiles\STMicroelectronics",
        "${env:ProgramFiles(x86)}\STMicroelectronics",
        "$env:LOCALAPPDATA\Programs\STM32CubeIDE",
        "C:\ST", "D:\CubeIDE", "C:\CubeIDE", "E:\CubeIDE"
    ) | Where-Object { $_ -and (Test-Path $_) }

    foreach ($root in $roots) {
        $found = Get-ChildItem $root -Recurse -Filter "STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue |
                 Select-Object -First 1
        if ($found) { $Cli = $found.FullName; break }
    }
}
if ([string]::IsNullOrWhiteSpace($Cli) -or -not (Test-Path $Cli)) {
    Write-Host "STM32_Programmer_CLI.exe not found; pass -Cli <path>" -ForegroundColor Red
    exit 2
}

$visible = @([System.IO.Ports.SerialPort]::GetPortNames())
if ($visible -notcontains $Port) {
    Write-Host ("{0} is not present. Plug the USB data cable in first." -f $Port) -ForegroundColor Red
    Write-Host ("  ports visible now: {0}" -f ($visible -join ' ')) -ForegroundColor Yellow
    exit 2
}

$sp = New-Object System.IO.Ports.SerialPort($Port, 115200, "None", 8, "One")
$sp.ReadTimeout = 300
try { $sp.Open() } catch {
    Write-Host ("cannot open {0}: {1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    exit 2
}
$sp.DiscardInBuffer()

Write-Host "resetting the target over SWD (no programming) ..." -ForegroundColor Cyan
$reset = & $Cli -c port=SWD mode=UR -rst 2>&1 | Out-String
if ($reset -match 'No STM32 target found|No debug probe') {
    Write-Host "cannot reach the target:" -ForegroundColor Red
    Write-Host $reset -ForegroundColor DarkGray
    $sp.Close(); $sp.Dispose()
    exit 2
}

$sb = New-Object System.Text.StringBuilder
$deadline = (Get-Date).AddSeconds($Seconds)
try {
    while ((Get-Date) -lt $deadline) {
        $chunk = ""
        try { $chunk = $sp.ReadExisting() } catch { }
        if ($chunk) { [void]$sb.Append($chunk); Write-Host -NoNewline $chunk }
        else { Start-Sleep -Milliseconds 40 }
    }
} finally {
    $sp.Close(); $sp.Dispose()
}

$text = $sb.ToString()
Write-Host ""
Write-Host "--- verdict -------------------------------------------------" -ForegroundColor Yellow

$frame = [regex]::Match($text, '\[drive\] frame: (.*)')
$ready = [regex]::Match($text, 'Part2 ready (.*)')

if ($frame.Success) {
    $f = $frame.Groups[1].Value.Trim()
    if ($f -match 'REVERSED') {
        Write-Host ("  drive frame : {0}" -f $f) -ForegroundColor Green
        Write-Host "                the car is placed DRIVEN-WHEELS-FIRST" -ForegroundColor DarkGray
    } else {
        Write-Host ("  drive frame : {0}" -f $f) -ForegroundColor Yellow
        Write-Host "                the car is placed CASTER-FIRST (pre 11-19 layout)" -ForegroundColor DarkGray
    }
} else {
    Write-Host "  drive frame : NOT SEEN (banner lost - run again)" -ForegroundColor Yellow
}

if ($ready.Success) {
    $r = $ready.Groups[1].Value.Trim()
    Write-Host ("  startup     : {0}" -f $r)
    if ($r -match 'imu=0') { Write-Host "  IMU         : ok" -ForegroundColor Green }
    else { Write-Host "  IMU         : FAILED - power-cycle the car and look again" -ForegroundColor Red }
    if ($r -match 'oled=0') { Write-Host "  OLED        : ok" -ForegroundColor Green }
    else { Write-Host "  OLED        : FAILED" -ForegroundColor Red }
} else {
    Write-Host "  startup     : NOT SEEN (banner lost - run again)" -ForegroundColor Yellow
}

if (($frame.Success -eq $false) -and ($ready.Success -eq $false)) { exit 3 }
exit 0
