# nav_run.ps1
#
# Role 6 evidence: run the car to a commanded (x, y) and record the endpoint
# error, automatically, one target at a time.
#
# WHY THIS EXISTS
# ---------------
# The scoring role "Bluetooth navigation" is one mark and it is earned with
# NUMBERS: three runs, each with an endpoint error and a heading error.  Doing
# that by hand means starting a run over one COM port, watching the 1 Hz block on
# another, guessing when the car has arrived, copying 'e' by hand and doing the
# vector arithmetic for each of the three targets.  This script keeps both links
# open, waits for the firmware's own arrival counters, reads the pose back and
# fills in the table.
#
#   debug link  COM8  115200   [1s] status lines, 'c' (zero), 'e' (read pose)
#   BT link     COM10 115200   "G <x> <y>" commands, one per line
#
# Three pitfalls this script is built around, all of them measured on 2026-11-17
# (HANDOVER 12.2):
#   * a STOP sent after the command arrives late and wipes the new path (pit E),
#     so this script NEVER sends 'S'; it waits for the arrival counters instead
#     and keeps the BT port open until it exits;
#   * the debug console is polled one byte per 50 ms, so keys are spaced out;
#   * arrival is judged from "PATH: g<generated> r<reached>" plus an empty
#     Q_Path, not from a stopwatch.
#
# Usage (or use nav_run.cmd):
#     tools\nav_run.ps1                      the three acceptance targets
#     tools\nav_run.ps1 -Only 0,1400         just the first one
#     tools\nav_run.ps1 -PauseSec 20          all three, with time to carry the car back
#     tools\nav_run.ps1 -Simulate            no hardware; proves the logic only
#     tools\nav_run.ps1 -SelfTest            parser check, no hardware
[CmdletBinding()]
param(
    [string]$DebugPort = 'COM8',
    [string]$BtPort = 'COM10',
    [int]$TimeoutSec = 60,
    [int]$PauseSec = 0,
    [string]$Only = '',
    [string]$Csv = '',
    [switch]$Simulate,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'

$KEY_GAP_MS = 90                   # console poll rate, see app_rtos.c
$ARRIVE_SETTLE_MS = 700            # after r == g, let the car come to rest
$TARGETS = @(
    @{ X = 0;     Y = 1400 },
    @{ X = -700;  Y = 700 },
    @{ X = 700;   Y = -700 }
)
$POSE_INIT_DEG = 90.0              # pose.c POSE_IMU_TO_NAV_DEG: boot heading is +90

$script:dbg = $null
$script:bt = $null
$script:buf = ''
$script:lines = New-Object System.Collections.ArrayList
$script:log = New-Object System.Collections.ArrayList

# ---- simulator -------------------------------------------------------------
$script:sim = $null

function Sim-Init {
    $script:sim = [pscustomobject]@{
        G = 0L; R = 0L; QP = 0L; Total = 0L; Pending = 0L
        Px = 0L; Py = 0L; H10 = [int64]($POSE_INIT_DEG * 10)
        Tx = 0L; Ty = 0L; NextStepMs = 0L
        Sw = [System.Diagnostics.Stopwatch]::StartNew()
    }
}

function Sim-Zero {
    # 'c' zeroes pose and encoders only - the path statistics keep accumulating,
    # which is exactly the trap the arrival test used to fall into.
    $script:sim.Px = 0; $script:sim.Py = 0; $script:sim.H10 = [int64]($POSE_INIT_DEG * 10)
}

function Sim-Command([string]$Line) {
    $m = [regex]::Match($Line, '^G\s+(-?\d+)\s+(-?\d+)')
    if (-not $m.Success) { return }
    $x = [int64]$m.Groups[1].Value
    $y = [int64]$m.Groups[2].Value
    $dist = [Math]::Sqrt([double]($x * $x + $y * $y))
    $script:sim.Tx = $x
    $script:sim.Ty = $y
    # G/R are cumulative since boot, exactly like path.c's counters.
    $script:sim.Total = [int64][Math]::Ceiling($dist / 100.0)
    $script:sim.Pending = $script:sim.Total
    $script:sim.G += $script:sim.Total
    $script:sim.QP = $script:sim.Pending
    $script:sim.NextStepMs = $script:sim.Sw.ElapsedMilliseconds + 250
    [void]$script:log.Add(('[sim] G {0} {1} -> {2} waypoints (cumulative g={3})' -f $x, $y, $script:sim.Total, $script:sim.G))
}

function Sim-Tick {
    # One waypoint every 250 ms; the last one lands with a small, deliberately
    # injected error so the error arithmetic has something to eat.  Status lines
    # keep coming after the run too, like the real 1 Hz block - the arrival test
    # needs to see the settled state twice.
    $now = $script:sim.Sw.ElapsedMilliseconds
    if ($now -lt $script:sim.NextStepMs) { return }

    if ($script:sim.Pending -gt 0) {
        $script:sim.Pending--
        $script:sim.R++
        $doneFrac = 1.0 - ([double]$script:sim.Pending / [double]$script:sim.Total)
        $script:sim.Px = [int64][Math]::Round($script:sim.Tx * $doneFrac)
        $script:sim.Py = [int64][Math]::Round($script:sim.Ty * $doneFrac)
        if ($script:sim.Pending -eq 0) {
            $script:sim.Px += 6
            $script:sim.Py -= 4
            $script:sim.H10 += 12
        }
        $script:sim.QP = $script:sim.Pending
        $script:sim.NextStepMs = $now + 250
    } else {
        $script:sim.NextStepMs = $now + 500
    }
    Emit-Status
}

function Emit-Status {
    $line = ('[1s] P: eh{0}.{1} ex{2} ey{3} | QP: {4} | PATH: g{5} r{6}' -f
        [Math]::Floor($script:sim.H10 / 10), [Math]::Abs($script:sim.H10 % 10),
        $script:sim.Px, $script:sim.Py, $script:sim.QP, $script:sim.G, $script:sim.R)
    [void]$script:lines.Add($line)
    [void]$script:log.Add($line)
}

function Emit-Enc {
    $line = ('[enc] L=0 R=0 cnt | rpm 0/0 | x={0} y={1} | h={2} gyro={2} (x0.1deg)' -f
        $script:sim.Px, $script:sim.Py, $script:sim.H10)
    [void]$script:lines.Add($line)
    [void]$script:log.Add($line)
}

# ---- plumbing --------------------------------------------------------------
function Pump {
    param([int]$Ms)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $Ms) {
        if ($script:sim) {
            Sim-Tick
            Start-Sleep -Milliseconds 10
            continue
        }
        $chunk = ''
        try { $chunk = $script:dbg.ReadExisting() } catch { $chunk = '' }
        if ($chunk.Length -gt 0) {
            $script:buf += $chunk
            while (($i = $script:buf.IndexOf("`n")) -ge 0) {
                $line = $script:buf.Substring(0, $i).TrimEnd("`r")
                $script:buf = $script:buf.Substring($i + 1)
                if ($line.Length -gt 0) {
                    [void]$script:lines.Add($line)
                    [void]$script:log.Add($line)
                }
            }
        } else {
            Start-Sleep -Milliseconds 10
        }
    }
}

function Send-Key([string]$Key) {
    if ($script:sim) {
        if ($Key -eq 'c') { Sim-Zero }
        if ($Key -eq 'e') { Emit-Enc }
    } else {
        $script:dbg.Write($Key)
    }
    Write-Host ("    >> debug '{0}'" -f $Key) -ForegroundColor DarkYellow
    Start-Sleep -Milliseconds $KEY_GAP_MS
}

function Send-Bt([string]$Line) {
    if ($script:sim) {
        Sim-Command $Line
    } else {
        $script:bt.Write($Line + "`n")
    }
    Write-Host ("    >> BT    '{0}'" -f $Line) -ForegroundColor DarkYellow
    Start-Sleep -Milliseconds 120
}

function Write-Head([string]$Text) {
    Write-Host ''
    Write-Host ('=== ' + $Text + ' ' + ('=' * [Math]::Max(0, 60 - $Text.Length))) -ForegroundColor Cyan
}

# ---- parsers ---------------------------------------------------------------
function Parse-Status([string]$Line) {
    $m = [regex]::Match($Line, '^\[1s\] P: eh(-?\d+(?:\.\d+)?) ex(-?\d+) ey(-?\d+) \| QP: (\d+) \| PATH: g(\d+) r(\d+)')
    if (-not $m.Success) { return $null }
    return [pscustomobject]@{
        Eh = [double]$m.Groups[1].Value
        Ex = [int64]$m.Groups[2].Value
        Ey = [int64]$m.Groups[3].Value
        QP = [int64]$m.Groups[4].Value
        G  = [int64]$m.Groups[5].Value
        R  = [int64]$m.Groups[6].Value
    }
}

function Parse-Enc([string]$Line) {
    $m = [regex]::Match($Line, 'L=(-?\d+)\s+R=(-?\d+)\s+cnt\s*\|\s*rpm\s+(-?\d+)/(-?\d+)\s*\|\s*x=(-?\d+)\s+y=(-?\d+)\s*\|\s*h=(-?\d+)')
    if (-not $m.Success) { return $null }
    return [pscustomobject]@{
        X  = [int64]$m.Groups[5].Value
        Y  = [int64]$m.Groups[6].Value
        H10 = [int64]$m.Groups[7].Value
    }
}

function Get-FinalPose {
    $script:lines.Clear()
    Send-Key 'e'
    $deadline = (Get-Date).AddMilliseconds(900)
    while ((Get-Date) -lt $deadline) {
        Pump -Ms 120
        foreach ($l in $script:lines) {
            if ($l -match '^\[enc\].*L=') {
                $r = Parse-Enc $l
                if ($r) { return $r }
            }
        }
    }
    return $null
}

function Get-BearingDeg([double]$X, [double]$Y) {
    # Pose frame (pose.c): heading is the standard maths angle, so +x is 0 deg and
    # +y is +90 deg - "facing forward at boot" is literally heading = 90.  The
    # bearing of a point is therefore atan2(y, x), NOT atan2(x, y): the two are
    # swapped, and getting it backwards puts a 90 deg error on every run.
    return [Math]::Atan2($Y, $X) * 180.0 / [Math]::PI
}

function Wrap180([double]$Deg) {
    while ($Deg -gt 180.0)  { $Deg -= 360.0 }
    while ($Deg -lt -180.0) { $Deg += 360.0 }
    return $Deg
}

# ---- one run ---------------------------------------------------------------
function Invoke-Run($Target) {
    $x = [int]$Target.X
    $y = [int]$Target.Y
    $dist = [Math]::Sqrt([double]($x * $x + $y * $y))

    Write-Head ('target ({0}, {1}) mm   distance {2:F0} mm' -f $x, $y, $dist)

    Send-Key 'c'                      # zero pose AND both encoder totals
    Pump -Ms 350
    # Baseline for the cumulative g counter, taken BEFORE the command is sent:
    # after the command g has already moved, so a baseline read afterwards can
    # never decrease and the "was it accepted?" test would never fire.
    $gBase = $null
    $script:lines.Clear()
    Pump -Ms 1200
    foreach ($l in $script:lines) {
        $s = Parse-Status $l
        if ($s) { $gBase = $s.G; break }
    }
    $script:lines.Clear()
    Send-Bt ('G {0} {1}' -f $x, $y)

    $t0 = Get-Date
    $deadline = $t0.AddSeconds($TimeoutSec)
    $last = $null
    $sawCommand = $false
    $done = $false
    $confirm = 0
    # WHAT "ARRIVED" MEANS.  Not r >= g: measured 2026-11-18, PATH: g/r are
    # CUMULATIVE since boot (path.c stats_add() only ever adds, nothing resets
    # them per path), so after a couple of runs g is far ahead of r forever and
    # a r >= g test never fires - the first version of this script reported a
    # 1.4 m run as complete 1.2 s in, then reported a perfect run as a timeout.
    # What is trustworthy is "Q_Path empty" plus "the pose is at the target":
    # the waypoint arrival radius is 30 mm (path.h PATH_ARRIVE_MM).
    $arriveTol = [Math]::Max(120.0, 0.10 * $dist)

    while ((Get-Date) -lt $deadline) {
        Pump -Ms 150
        foreach ($l in $script:lines) {
            $s = Parse-Status $l
            if ($s) {
                $last = $s
                if (($s.QP -gt 0) -or (($null -ne $gBase) -and ($s.G -gt $gBase))) { $sawCommand = $true }
                $dTgt = [Math]::Sqrt([double](($s.Ex - $x) * ($s.Ex - $x) + ($s.Ey - $y) * ($s.Ey - $y)))
                if (($s.QP -eq 0) -and ($dTgt -le $arriveTol) -and $sawCommand) {
                    $confirm++
                    if ($confirm -ge 2) { $done = $true; break }
                } else {
                    $confirm = 0
                }
            }
        }
        $script:lines.Clear()
        if ($done) { break }
        if (-not $sawCommand -and ((Get-Date) - $t0).TotalSeconds -gt 6) {
            throw 'the car never acknowledged the command (no new waypoints) - check the BT link'
        }
    }

    if (-not $done) {
        Write-Host ('  TIMEOUT after {0} s - sending S to stop the car' -f $TimeoutSec) -ForegroundColor Red
        Send-Bt 'S'
        Pump -Ms 600
    } else {
        Pump -Ms $ARRIVE_SETTLE_MS
    }

    $pose = Get-FinalPose
    if (-not $pose) { throw 'no pose readout after the run ([enc] line missing)' }

    $dx = [double]($x - $pose.X)
    $dy = [double]($y - $pose.Y)
    $err = [Math]::Sqrt($dx * $dx + $dy * $dy)
    $pct = if ($dist -gt 1.0) { 100.0 * $err / $dist } else { 0.0 }
    $bearing = Get-BearingDeg $x $y
    $headErr = Wrap180 (($pose.H10 / 10.0) - $bearing)
    $secs = ((Get-Date) - $t0).TotalSeconds

    Write-Host ''
    Write-Host ('  commanded   ({0}, {1}) mm' -f $x, $y)
    Write-Host ('  measured    ({0}, {1}) mm   heading {2:F1} deg' -f $pose.X, $pose.Y, ($pose.H10 / 10.0))
    Write-Host ('  endpoint    dx {0:+0;-0;0} mm   dy {1:+0;-0;0} mm   |err| {2:F0} mm  ({3:F1} % of {4:F0} mm)' -f $dx, $dy, $err, $pct, $dist) -ForegroundColor White
    Write-Host ('  heading     expected {0:F1} deg   error {1:+0.0;-0.0;0.0} deg' -f $bearing, $headErr)
    Write-Host ('  duration    {0:F1} s  (v_max 120 mm/s -> {1:F1} s ideal, turning included)' -f $secs, ($dist / 120.0))
    if (-not $done) { Write-Host '  !! this run timed out: the numbers above are NOT a completed run' -ForegroundColor Red }

    return [pscustomobject]@{
        X = $x; Y = $y; Mx = $pose.X; My = $pose.Y
        Dx = $dx; Dy = $dy; Err = $err; Pct = $pct
        Heading = ($pose.H10 / 10.0); Bearing = $bearing; HeadErr = $headErr
        Secs = $secs; Complete = $done
    }
}

# ---- self test -------------------------------------------------------------
function Invoke-SelfTest {
    Write-Head 'self test (parsers and geometry, no hardware)'
    $fail = 0

    $s = Parse-Status '[1s] P: eh90.0 ex0 ey489 | QP: 3 | PATH: g14 r11'
    if (-not $s) { Write-Host '  FAIL status: did not parse' -ForegroundColor Red; $fail++ }
    else {
        if ($s.Eh -ne 90.0) { Write-Host '  FAIL status eh' -ForegroundColor Red; $fail++ }
        if ($s.Ex -ne 0)    { Write-Host '  FAIL status ex' -ForegroundColor Red; $fail++ }
        if ($s.Ey -ne 489)  { Write-Host '  FAIL status ey' -ForegroundColor Red; $fail++ }
        if ($s.QP -ne 3)    { Write-Host '  FAIL status QP' -ForegroundColor Red; $fail++ }
        if ($s.G -ne 14)    { Write-Host '  FAIL status g' -ForegroundColor Red; $fail++ }
        if ($s.R -ne 11)    { Write-Host '  FAIL status r' -ForegroundColor Red; $fail++ }
    }

    $s2 = Parse-Status '[1s] P: eh-45.0 ex-700 ey700 | QP: 0 | PATH: g10 r10'
    if (-not $s2) { Write-Host '  FAIL status negative: did not parse' -ForegroundColor Red; $fail++ }
    else {
        if ($s2.Eh -ne -45.0) { Write-Host '  FAIL status negative eh' -ForegroundColor Red; $fail++ }
        if ($s2.Ex -ne -700)  { Write-Host '  FAIL status negative ex' -ForegroundColor Red; $fail++ }
    }

    if (Parse-Status '[enc] L=0 R=0 cnt | rpm 0/0 | x=0 y=0 | h=900 gyro=900 (x0.1deg)') {
        Write-Host '  FAIL status: accepted an [enc] line' -ForegroundColor Red; $fail++
    }

    $e = Parse-Enc '[enc] L=1320 R=-1320 cnt | rpm 83/82 | x=-700 y=700 | h=-451 gyro=-449 (x0.1deg)'
    if (-not $e) { Write-Host '  FAIL enc: did not parse' -ForegroundColor Red; $fail++ }
    else {
        if ($e.X -ne -700) { Write-Host '  FAIL enc x' -ForegroundColor Red; $fail++ }
        if ($e.H10 -ne -451) { Write-Host '  FAIL enc h' -ForegroundColor Red; $fail++ }
    }

    $b = Get-BearingDeg 0 1400
    if ([Math]::Abs($b - 90.0) -gt 0.01) { Write-Host ('  FAIL bearing +y = {0}' -f $b) -ForegroundColor Red; $fail++ }
    $b = Get-BearingDeg -700 700
    if ([Math]::Abs($b - 135.0) -gt 0.01) { Write-Host ('  FAIL bearing -x+y = {0}' -f $b) -ForegroundColor Red; $fail++ }
    $b = Get-BearingDeg 700 -700
    if ([Math]::Abs($b - (-45.0)) -gt 0.01) { Write-Host ('  FAIL bearing +x-y = {0}' -f $b) -ForegroundColor Red; $fail++ }

    if ([Math]::Abs((Wrap180 190.0) - (-170.0)) -gt 0.01) { Write-Host '  FAIL wrap180 +190' -ForegroundColor Red; $fail++ }
    if ([Math]::Abs((Wrap180 -190.0) - 170.0) -gt 0.01)   { Write-Host '  FAIL wrap180 -190' -ForegroundColor Red; $fail++ }

    Write-Host ''
    if ($fail -eq 0) { Write-Host '  self test: all checks passed' -ForegroundColor Green; return 0 }
    Write-Host ('  self test: {0} check(s) FAILED' -f $fail) -ForegroundColor Red
    return 1
}

# ---- main ------------------------------------------------------------------
if ($SelfTest) { exit (Invoke-SelfTest) }

if ($Only) {
    $parts = $Only -split ','
    if ($parts.Count -ne 2) { Write-Host '-Only wants "x,y"' -ForegroundColor Red; exit 2 }
    $TARGETS = @(@{ X = [int]$parts[0].Trim(); Y = [int]$parts[1].Trim() })
}

$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$logPath = Join-Path $logDir ('nav_' + $stamp + '.txt')
if (-not $Csv) { $Csv = Join-Path $logDir ('nav_' + $stamp + '.csv') }

Write-Host ''
Write-Host ('nav_run  |  debug {0} / BT {1}  |  {2} target(s)' -f $DebugPort, $BtPort, $TARGETS.Count) -ForegroundColor White
Write-Host ('  log -> {0}' -f $logPath) -ForegroundColor DarkGray
Write-Host ('  csv -> {0}' -f $Csv) -ForegroundColor DarkGray
Write-Host '  The car starts from wherever it is; "measured" is in the pose frame that'
Write-Host '  "c" just zeroed, so this assumes the car is placed on the start point'
Write-Host '  facing +y before each run.' -ForegroundColor DarkGray
if ($Simulate) {
    Write-Host ''
    Write-Host '  *** SIMULATION - no hardware, no measurement.  Numbers are synthetic.' -ForegroundColor Magenta
    Sim-Init
} else {
    try {
        $script:dbg = New-Object System.IO.Ports.SerialPort($DebugPort, 115200, 'None', 8, 'One')
        $script:dbg.ReadTimeout = 200
        $script:dbg.DtrEnable = $false
        $script:dbg.RtsEnable = $false
        $script:dbg.Open()
        Start-Sleep -Milliseconds 300     # let the VCP finish initialising
        $script:dbg.DiscardInBuffer()
    } catch {
        Write-Host ('  cannot open {0}: {1}' -f $DebugPort, $_.Exception.Message) -ForegroundColor Red
        exit 2
    }
    try {
        $script:bt = New-Object System.IO.Ports.SerialPort($BtPort, 115200, 'None', 8, 'One')
        $script:bt.ReadTimeout = 200
        $script:bt.Open()
    } catch {
        Write-Host ('  cannot open {0}: {1}' -f $BtPort, $_.Exception.Message) -ForegroundColor Red
        Write-Host '  Is the HC-04 paired and switched on?  COM10 was the SPP port on 2026-11-17.'
        try { $script:dbg.Close() } catch { }
        exit 2
    }
}

$results = New-Object System.Collections.ArrayList
$rc = 0
try {
    $k = 0
    foreach ($t in $TARGETS) {
        $k++
        [void]$results.Add((Invoke-Run $t))
        Pump -Ms 400
        if (($PauseSec -gt 0) -and ($k -lt $TARGETS.Count)) {
            # The car is now somewhere else; every run zeroes the pose with 'c',
            # so it must be carried back to the start point before the next one.
            Write-Host ''
            for ($s = $PauseSec; $s -gt 0; $s--) {
                Write-Host ("  put the car back on the start mark, facing +y ... {0}s" -f $s) -ForegroundColor Yellow
                Start-Sleep -Seconds 1
            }
        }
    }
} catch {
    Write-Host ('  ERROR: {0}' -f $_.Exception.Message) -ForegroundColor Red
    $rc = 4
} finally {
    # No 'S' here unless a run timed out: a late STOP would wipe the next path
    # (HANDOVER 12.2, pit E).  The car stops itself when the waypoint queue is
    # empty.  Flush and close both links.
    try { if ($script:dbg -and $script:dbg.IsOpen) { $script:dbg.Close(); $script:dbg.Dispose() } } catch { }
    try { if ($script:bt -and $script:bt.IsOpen) { Start-Sleep -Milliseconds 200; $script:bt.Close(); $script:bt.Dispose() } } catch { }

    $body = "# nav_run @ $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')`r`n" +
            "# full debug capture, unfiltered`r`n`r`n" +
            (($script:log | ForEach-Object { $_.ToString() }) -join "`r`n") + "`r`n"
    [System.IO.File]::WriteAllText($logPath, $body, (New-Object System.Text.UTF8Encoding($false)))
}

if ($results.Count -gt 0) {
    Write-Head 'role 6 evidence table'
    $rows = @('| # | commanded (mm) | measured (mm) | |err| | err % | heading err | complete |',
              '|---|---|---|---|---|---|---|')
    $i = 0
    foreach ($r in $results) {
        $i++
        $rows += ('| {0} | ({1}, {2}) | ({3}, {4}) | {5:F0} mm | {6:F1} % | {7:+0.0;-0.0;0.0} deg | {8} |' -f
            $i, $r.X, $r.Y, $r.Mx, $r.My, $r.Err, $r.Pct, $r.HeadErr, $(if ($r.Complete) { 'yes' } else { 'TIMEOUT' }))
    }
    $rows | ForEach-Object { Write-Host $_ }

    $csvLines = @('run,x_cmd,y_cmd,x_meas,y_meas,dx,dy,err_mm,err_pct,heading_deg,heading_err_deg,seconds,complete')
    $i = 0
    foreach ($r in $results) {
        $i++
        $csvLines += ('{0},{1},{2},{3},{4},{5},{6},{7:F1},{8:F2},{9:F1},{10:F1},{11:F1},{12}' -f
            $i, $r.X, $r.Y, $r.Mx, $r.My, $r.Dx, $r.Dy, $r.Err, $r.Pct, $r.Heading, $r.HeadErr, $r.Secs, $r.Complete)
    }
    [System.IO.File]::WriteAllText($Csv, ($csvLines -join "`r`n") + "`r`n", (New-Object System.Text.UTF8Encoding($false)))
    Write-Host ''
    Write-Host ('  copy the table above into the report; csv -> {0}' -f $Csv) -ForegroundColor DarkGray
}

exit $rc
