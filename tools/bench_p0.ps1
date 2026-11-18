# bench_p0.ps1
#
# P0 bench measurements for Block 4.5 Part 2, run from the PC over USART2.
#
# WHY THIS EXISTS
# ---------------
# The four P0 items in Docs/NEXT_SESSION.md 8 are all "drive the wheels and read
# numbers back".  Doing that by hand means: press a key, wait for the burst,
# press 'e', copy the line, subtract two 32-bit numbers in your head, repeat for
# four directions, and remember to zero the counters in between.  That is how
# yesterday's session lost an hour to a STOP that arrived late (HANDOVER 12.2,
# pit E).  This script drives the same console keys the firmware already has -
# it adds NO firmware code - and does the arithmetic and the verdict.
#
#   battery   P0 #1  is the pack able to turn the motors at 20 % duty?
#   symmetry  P0 #3  right wheel counts versus left, forward vs reverse
#   straight  P0 #2  open-loop straight line: drift per metre, A/B comparable
#
# WHAT IT CANNOT DO
# -----------------
# It cannot measure battery voltage (no ADC wired yet) and it cannot measure
# lateral offset with a tape.  Both stay manual; the script prints the fields to
# fill in.  Battery health is inferred the way HANDOVER 12.4 established it:
# a healthy pack turns the wheels at 20 % duty, a flat one does not.
#
# The console is polled one byte per 50 ms slot (app_rtos.c console_poll), so
# keys MUST be spaced out; Send-Key does that and is the only way to talk to it.
#
# Usage (or use bench_p0.cmd):
#     tools\bench_p0.ps1 -Test battery       wheels OFF the ground
#     tools\bench_p0.ps1 -Test symmetry      wheels OFF the ground
#     tools\bench_p0.ps1 -Test straight -Bursts 5     on the 1 m tape
#     tools\bench_p0.ps1 -Test all
#     tools\bench_p0.ps1 -SelfTest           parser check, no hardware
[CmdletBinding()]
param(
    [string]$Port = 'COM8',
    [ValidateSet('battery', 'symmetry', 'straight', 'all')]
    [string]$Test = 'all',
    [int]$Bursts = 5,
    [double]$TapeDriftMm = [double]::NaN,
    [switch]$SelfTest,
    [switch]$Simulate
)

$ErrorActionPreference = 'Stop'

# Firmware constants these timings mirror (app_rtos.c, bench block).
$BENCH_HOLD_MS = 1500      # BENCH_HOLD_MS: one 'f' burst
$BENCH_GAP_MS = 1200       # re-issue before the burst lapses -> continuous drive
$KEY_GAP_MS = 90           # console samples 1 byte per 50 ms slot
$FULL_PACK_RPM_20PCT = 83  # HANDOVER 12.4: measured at 20 % duty, full pack

$script:sp = $null
$script:buf = ''
$script:lines = New-Object System.Collections.ArrayList
$script:log = New-Object System.Collections.ArrayList

# ---- simulator (see -Simulate) --------------------------------------------
# A fake device that answers the same keys with the same line formats.  It
# exists so the burst arithmetic, the ratio table and the verdict logic can be
# exercised on a desk with the car switched off; it is NOT a measurement and
# every number it prints is synthetic.
$script:sim = $null
$script:simQ = New-Object System.Collections.ArrayList

function Sim-Init {
    $script:sim = [pscustomobject]@{
        L = 0L; R = 0L; Y = 0L; X = 0L; H10 = 900L; RpmL = 0; RpmR = 0
    }
}

function Sim-Handle {
    # Injected fault, mirroring the 2026-11-17 measurements: the right wheel
    # counts ~0.50x going forward and ~4.7x in reverse - an encoder input that
    # counts edges in one direction only.
    param([string]$Key)
    $k = $Key.ToUpper()
    switch ($k) {
        'H' { [void]$script:simQ.Add('[cmd] s=stop lidar g=restart lidar f/b=both fwd/rev l/r=left/right only x=stop c=zero enc e=read enc h=help') }
        'C' {
            $script:sim.L = 0; $script:sim.R = 0; $script:sim.Y = 0; $script:sim.X = 0
            [void]$script:simQ.Add('[enc] zero requested (pose + both encoder totals)')
        }
        'E' {
            [void]$script:simQ.Add(('[enc] L={0} R={1} cnt | rpm {2}/{3} | x={4} y={5} | h={6} gyro={6} (x0.1deg)' -f
                $script:sim.L, $script:sim.R, $script:sim.RpmL, $script:sim.RpmR, $script:sim.X, $script:sim.Y, $script:sim.H10))
        }
        'X' { [void]$script:simQ.Add('[bench] stop requested') }
        'F' { Sim-Move 1650 825 83 42; [void]$script:simQ.Add('[bench] both forward 20% for 1500 ms') }
        'B' { Sim-Move -1650 -7755 -83 -390; [void]$script:simQ.Add('[bench] both reverse 20% for 1500 ms') }
        'L' { Sim-Move 1650 0 83 0; [void]$script:simQ.Add('[bench] LEFT channel only 20% for 1500 ms') }
        'R' { Sim-Move 0 825 0 42; [void]$script:simQ.Add('[bench] RIGHT channel only 20% for 1500 ms') }
        default { }
    }
}

function Sim-Move([int]$dL, [int]$dR, [int]$rpmL, [int]$rpmR) {
    $script:sim.L += $dL
    $script:sim.R += $dR
    $script:sim.RpmL = $rpmL
    $script:sim.RpmR = $rpmR
    # 0.1561 mm/count is the measured odometry scale (HANDOVER 12.3).
    $script:sim.Y += [int64][Math]::Round(0.5 * ($dL + $dR) * 0.1561)
    [void]$script:simQ.Add(('[1s] P: eh{0:F1} ex{1} ey{2} | QP: 0 | PATH: g10 r7' -f
        ($script:sim.H10 / 10.0), $script:sim.X, $script:sim.Y))
}

# ---------------------------------------------------------------------------
# plumbing
# ---------------------------------------------------------------------------
function Pump {
    # Read for Ms milliseconds, appending complete lines to $script:lines.
    param([int]$Ms)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $Ms) {
        if ($script:sim) {
            while ($script:simQ.Count -gt 0) {
                $line = [string]$script:simQ[0]
                $script:simQ.RemoveAt(0)
                [void]$script:lines.Add($line)
                [void]$script:log.Add($line)
            }
            Start-Sleep -Milliseconds 10
            continue
        }
        $chunk = ''
        try { $chunk = $script:sp.ReadExisting() } catch { $chunk = '' }
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

function Send-Key {
    # 90 ms between keys: console_poll() consumes at most one byte per 50 ms
    # slot and the UART has no FIFO, so a burst of bytes would be dropped.
    param([string]$Key)
    if ($script:sim) {
        Sim-Handle $Key
    } else {
        $script:sp.Write($Key)
    }
    Write-Host ("    >> '{0}'" -f $Key) -ForegroundColor DarkYellow
    Start-Sleep -Milliseconds $KEY_GAP_MS
}

function Write-Head([string]$Text) {
    Write-Host ''
    Write-Host ('=== ' + $Text + ' ' + ('=' * [Math]::Max(0, 60 - $Text.Length))) -ForegroundColor Cyan
}

# ---------------------------------------------------------------------------
# parsers (these are what -SelfTest exercises)
# ---------------------------------------------------------------------------
function Parse-Enc {
    # [enc] L=1320 R=-1320 cnt | rpm 83/82 | x=0 y=489 | h=900 gyro=899 (x0.1deg)
    param([string]$Line)
    $m = [regex]::Match($Line, 'L=(-?\d+)\s+R=(-?\d+)\s+cnt\s*\|\s*rpm\s+(-?\d+)/(-?\d+)\s*\|\s*x=(-?\d+)\s+y=(-?\d+)\s*\|\s*h=(-?\d+)\s+gyro=(-?\d+)')
    if (-not $m.Success) { return $null }
    return [pscustomobject]@{
        L      = [int64]$m.Groups[1].Value
        R      = [int64]$m.Groups[2].Value
        RpmL   = [int]$m.Groups[3].Value
        RpmR   = [int]$m.Groups[4].Value
        X      = [int64]$m.Groups[5].Value
        Y      = [int64]$m.Groups[6].Value
        H10    = [int64]$m.Groups[7].Value
        Gyro10 = [int64]$m.Groups[8].Value
    }
}

function Parse-Pose {
    # [1s] P: eh90.0 ex0 ey489 | QP: 0 | PATH: g10 r7
    param([string]$Line)
    $m = [regex]::Match($Line, '^\[1s\] P: eh(-?\d+(?:\.\d+)?) ex(-?\d+) ey(-?\d+)')
    if (-not $m.Success) { return $null }
    return [pscustomobject]@{
        Eh = [double]$m.Groups[1].Value
        Ex = [int64]$m.Groups[2].Value
        Ey = [int64]$m.Groups[3].Value
    }
}

# ---------------------------------------------------------------------------
# device access
# ---------------------------------------------------------------------------
function Get-Enc {
    # Ask for one '[enc] ...' readout.  Returns $null if the firmware is silent.
    param([int]$TimeoutMs = 800)
    $script:lines.Clear()
    Send-Key 'e'
    $deadline = (Get-Date).AddMilliseconds($TimeoutMs)
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

function Assert-Alive {
    # The one thing worth failing fast on: a target that prints nothing is a
    # power problem, not a software problem (measured 2026-11-18: 0.11 V).
    # Retried: the first byte written right after SerialPort.Open() is sometimes
    # swallowed while the VCP finishes initialising (measured 2026-11-18 - the
    # single-shot version reported a perfectly healthy board as dead).
    $reply = $null
    for ($try = 1; $try -le 3 -and -not $reply; $try++) {
        $script:lines.Clear()
        Send-Key 'h'
        Pump -Ms 700
        $reply = $script:lines | Where-Object { $_ -match '^\[cmd\]' } | Select-Object -First 1
    }
    if (-not $reply) {
        Write-Host ''
        Write-Host '  NO ANSWER on the debug console (3 tries).' -ForegroundColor Red
        Write-Host '  The MCU is not running.  Check, in this order:' -ForegroundColor Yellow
        Write-Host '    1. main power switch ON, pack connected and charged'
        Write-Host '    2. tools\flash_and_monitor.cmd -Probe   (expect a voltage, not 0.1 V)'
        Write-Host '    3. only then suspect the firmware'
        return $false
    }
    Write-Host ('    console alive: ' + $reply.Trim()) -ForegroundColor DarkGray
    return $true
}

function New-Burst {
    # Zero the counters, run one open-loop burst, read the totals back.
    # The burst length is fixed by the firmware, so host timing only has to be
    # "long enough" - which is why a 90 ms per key link is still good enough.
    param([string]$Key, [string]$Label)
    Send-Key 'c'
    Pump -Ms 160
    $before = Get-Enc
    if (-not $before) { throw "no '[enc]' reply before burst '$Label' - firmware stopped answering" }
    Send-Key $Key
    Pump -Ms ($BENCH_HOLD_MS + 420)
    $after = Get-Enc
    if (-not $after) { throw "no '[enc]' reply after burst '$Label'" }
    Send-Key 'x'
    Pump -Ms 120
    return [pscustomobject]@{
        Label = $Label
        dL    = $after.L - $before.L
        dR    = $after.R - $before.R
        RpmL  = $after.RpmL
        RpmR  = $after.RpmR
    }
}

function Format-Ratio([double]$Num, [double]$Den) {
    if ([Math]::Abs($Den) -lt 1.0) { return 'n/a' }
    return ('{0:F2}' -f ($Num / $Den))
}

# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
function Test-Battery {
    Write-Head 'P0 #1  battery health  (wheels OFF the ground)'
    Write-Host '  Healthy pack: 20 % duty turns the wheels (HANDOVER 12.4 measured 83 rpm).'
    Write-Host '  Flat pack:    20 % duty is a dead zone - wheels do not move at all.'
    $b = New-Burst -Key 'f' -Label 'both forward'
    Write-Host ''
    Write-Host ('  counts  L={0,7}  R={1,7}   (1 wheel rev = 1320 counts)' -f $b.dL, $b.dR)
    Write-Host ('  rpm     L={0,7}  R={1,7}' -f $b.RpmL, $b.RpmR)

    $best = [Math]::Max([Math]::Abs($b.RpmL), [Math]::Abs($b.RpmR))
    Write-Host ''
    if ($best -ge ($FULL_PACK_RPM_20PCT * 0.72)) {
        Write-Host ('  VERDICT: pack fine at 20 % duty ({0} rpm vs {1} rpm reference).' -f $best, $FULL_PACK_RPM_20PCT) -ForegroundColor Green
    } elseif ($best -ge 20) {
        Write-Host ('  VERDICT: pack WEAK ({0} rpm vs {1} rpm reference). Charge before any tuning.' -f $best, $FULL_PACK_RPM_20PCT) -ForegroundColor Yellow
    } else {
        Write-Host ('  VERDICT: 20 % duty is a DEAD ZONE ({0} rpm). Charge, then repeat.' -f $best) -ForegroundColor Red
        Write-Host '           If the pack is known-charged, this is not the battery: read CCR over' -ForegroundColor Red
        Write-Host '           SWD (0x40000034 / 0x40000834) to prove the PWM is really non-zero.' -ForegroundColor Red
    }
    if (([Math]::Abs($b.dL) -lt 50) -xor ([Math]::Abs($b.dR) -lt 50)) {
        Write-Host '  NOTE: one wheel barely moved - that is a channel problem, not a battery one.' -ForegroundColor Yellow
    }
}

function Test-Symmetry {
    Write-Head 'P0 #3  left/right symmetry  (wheels OFF the ground)'
    Write-Host '  Same 20 % duty in all four cases. The LEFT wheel is the control:'
    Write-Host '  if only the right counts misbehave, its encoder inputs are suspect.'
    $runs = @(
        @{ Key = 'f'; Label = 'both forward' },
        @{ Key = 'b'; Label = 'both reverse' },
        @{ Key = 'l'; Label = 'left  only   ' },
        @{ Key = 'r'; Label = 'right only   ' }
    )
    $res = @{}
    foreach ($r in $runs) {
        $b = New-Burst -Key $r.Key -Label $r.Label
        $res[$r.Key] = $b
        Write-Host ('  {0}  dL={1,7}  dR={2,7}   rpm {3,4}/{4,4}' -f $r.Label, $b.dL, $b.dR, $b.RpmL, $b.RpmR)
    }

    Write-Host ''
    Write-Host '  --- forward vs reverse, per wheel (|rev|/|fwd| should be ~1.00) ---'
    $fL = [Math]::Abs([double]$res['f'].dL); $fR = [Math]::Abs([double]$res['f'].dR)
    $bL = [Math]::Abs([double]$res['b'].dL); $bR = [Math]::Abs([double]$res['b'].dR)
    $lL = [Math]::Abs([double]$res['l'].dL); $rR = [Math]::Abs([double]$res['r'].dR)
    Write-Host ('  LEFT   fwd {0,7}  rev {1,7}  rev/fwd {2}' -f $fL, $bL, (Format-Ratio $bL $fL)) -ForegroundColor White
    Write-Host ('  RIGHT  fwd {0,7}  rev {1,7}  rev/fwd {2}' -f $fR, $bR, (Format-Ratio $bR $fR)) -ForegroundColor White
    Write-Host ('  single-channel check: left-only dL {0,7}  right-only dR {1,7}' -f $lL, $rR)
    Write-Host ''
    Write-Host '  --- right/left ratio in the same direction (should be ~1.00) ---'
    Write-Host ('  forward  R/L {0}   reverse  R/L {1}' -f (Format-Ratio $fR $fL), (Format-Ratio $bR $bL)) -ForegroundColor White
    Write-Host ''
    Write-Host '  HOW TO READ IT:' -ForegroundColor Yellow
    Write-Host '    both ratios near 1.00                     -> wheels and encoders agree'
    Write-Host '    ONE wheel''s rev/fwd far from 1.00        -> that encoder counts edges in one'
    Write-Host '                                                 direction only: reseat/ohm its A and'
    Write-Host '                                                 B wires before touching any gain'
    Write-Host '    right ratio off but left ratio fine        -> right encoder input, not the motor'
    Write-Host '    Look at the wheels as well: if the two look the same speed while the'
    Write-Host '    counts differ by 2x or more, it is the ENCODER, not the motor.'
    Write-Host ''
    Write-Host '  RECORD: do both wheels look the same speed?  yes / no  ->  ______' -ForegroundColor Yellow
}

function Test-Straight {
    Write-Head ('P0 #2  open-loop straight line, {0} bursts (on the 1 m tape)' -f $Bursts)
    Write-Host '  Same command every burst, so "front wheels lifted" and "on the ground"'
    Write-Host '  are directly comparable: this is the A/B half of the drift test.'
    Send-Key 'c'
    Pump -Ms 200
    $start = Get-Enc
    if (-not $start) { throw 'no [enc] reply at the start of the straight run' }

    $poses = New-Object System.Collections.ArrayList
    $script:lines.Clear()
    for ($i = 1; $i -le $Bursts; $i++) {
        Send-Key 'f'             # re-issues the burst: drive is continuous
        # Read whatever arrived during the gap; keep the pose lines.
        Pump -Ms ($BENCH_GAP_MS - $KEY_GAP_MS)
        foreach ($l in $script:lines) {
            $p = Parse-Pose $l
            if ($p) { [void]$poses.Add($p) }
        }
        $script:lines.Clear()
        Write-Host ('    burst {0}/{1}' -f $i, $Bursts) -ForegroundColor DarkGray
    }
    Send-Key 'x'
    Pump -Ms 400
    $end = Get-Enc
    if (-not $end) { throw 'no [enc] reply at the end of the straight run' }

    $dL = $end.L - $start.L
    $dR = $end.R - $start.R
    $dist = $end.Y - $start.Y            # +y is straight ahead in this pose frame
    $lat = $end.X - $start.X
    $head = ($end.Gyro10 - $start.Gyro10) / 10.0

    Write-Host ''
    Write-Host ('  encoder  dL={0}  dR={1}   (R/L {2})' -f $dL, $dR, (Format-Ratio ([double]$dR) ([double]$dL)))
    Write-Host ('  pose     forward {0} mm   lateral {1} mm   heading {2} deg' -f $dist, $lat, $head)
    if ([Math]::Abs($dist) -ge 200) {
        Write-Host ('  drift    {0:F1} mm per metre (firmware view)' -f (1000.0 * $lat / $dist)) -ForegroundColor White
    } else {
        Write-Host '  drift    n/a: travelled less than 200 mm, no useful ratio' -ForegroundColor Yellow
    }
    if ($poses.Count -gt 0) {
        Write-Host ('  pose samples used: {0}, eh {1} -> {2}' -f $poses.Count, $poses[0].Eh, $poses[$poses.Count - 1].Eh)
    }

    Write-Host ''
    Write-Host '  FILL IN FROM THE TAPE (the firmware view above is dead-reckoned, the' -ForegroundColor Yellow
    Write-Host '  tape is the ground truth):' -ForegroundColor Yellow
    Write-Host '    lateral offset at the far end   ______ mm'
    Write-Host '    travelled distance              ______ mm'
    if (-not [double]::IsNaN($TapeDriftMm)) {
        Write-Host ('    tape says {0} mm' -f $TapeDriftMm) -ForegroundColor White
    }
}

# ---------------------------------------------------------------------------
# self test - no hardware
# ---------------------------------------------------------------------------
function Invoke-SelfTest {
    Write-Head 'self test (parsers only, no hardware)'
    $fail = 0

    $e = Parse-Enc '[enc] L=1320 R=-1320 cnt | rpm 83/82 | x=0 y=489 | h=900 gyro=899 (x0.1deg)'
    if (-not $e) { Write-Host '  FAIL enc: did not parse' -ForegroundColor Red; $fail++ }
    else {
        if ($e.L -ne 1320)   { Write-Host '  FAIL enc L' -ForegroundColor Red; $fail++ }
        if ($e.R -ne -1320)  { Write-Host '  FAIL enc R' -ForegroundColor Red; $fail++ }
        if ($e.RpmL -ne 83)  { Write-Host '  FAIL enc rpmL' -ForegroundColor Red; $fail++ }
        if ($e.H10 -ne 900)  { Write-Host '  FAIL enc h' -ForegroundColor Red; $fail++ }
        if ($e.Gyro10 -ne 899) { Write-Host '  FAIL enc gyro' -ForegroundColor Red; $fail++ }
    }

    if (Parse-Enc '[enc] zero requested (pose + both encoder totals)') { Write-Host '  FAIL enc: accepted the zero-ack line' -ForegroundColor Red; $fail++ }
    if (Parse-Enc '[1s] P: eh90.0 ex0 ey489 | QP: 0 | PATH: g10 r7') { Write-Host '  FAIL enc: accepted a pose line' -ForegroundColor Red; $fail++ }

    $p = Parse-Pose '[1s] P: eh-12.5 ex-34 ey489 | QP: 6 | PATH: g10 r7'
    if (-not $p) { Write-Host '  FAIL pose: did not parse' -ForegroundColor Red; $fail++ }
    else {
        if ($p.Eh -ne -12.5) { Write-Host '  FAIL pose eh' -ForegroundColor Red; $fail++ }
        if ($p.Ex -ne -34)   { Write-Host '  FAIL pose ex' -ForegroundColor Red; $fail++ }
        if ($p.Ey -ne 489)   { Write-Host '  FAIL pose ey' -ForegroundColor Red; $fail++ }
    }

    if ((Format-Ratio 100 200) -ne '0.50') { Write-Host '  FAIL ratio format' -ForegroundColor Red; $fail++ }
    if ((Format-Ratio 10 0) -ne 'n/a')     { Write-Host '  FAIL ratio div0' -ForegroundColor Red; $fail++ }

    Write-Host ''
    if ($fail -eq 0) { Write-Host '  self test: all parser checks passed' -ForegroundColor Green; return 0 }
    Write-Host ('  self test: {0} check(s) FAILED' -f $fail) -ForegroundColor Red
    return 1
}

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
if ($SelfTest) {
    exit (Invoke-SelfTest)
}

$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$logPath = Join-Path $logDir (('bench_{0:yyyyMMdd_HHmmss}.txt' -f (Get-Date)))

Write-Host ''
Write-Host ('bench_p0  |  {0} @ 115200 8N1  |  test={1}' -f $Port, $Test) -ForegroundColor White
Write-Host ('  log -> {0}' -f $logPath) -ForegroundColor DarkGray
if ($Simulate) {
    Write-Host ''
    Write-Host '  *** SIMULATION - no hardware, no measurement.  Every number below is' -ForegroundColor Magenta
    Write-Host '  *** synthetic and exists only to prove the script and the verdict logic.' -ForegroundColor Magenta
    Sim-Init
} elseif ($Test -ne 'straight') {
    Write-Host '  !! Lift the wheels off the ground: a 20 % burst on the floor drives the car.' -ForegroundColor Yellow
}

if (-not $Simulate) {
    try {
        $script:sp = New-Object System.IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
        $script:sp.ReadTimeout = 200
        $script:sp.DtrEnable = $false      # DTR/RTS can pulse NRST on this adapter
        $script:sp.RtsEnable = $false
        $script:sp.Open()
        Start-Sleep -Milliseconds 300     # let the VCP finish initialising
        $script:sp.DiscardInBuffer()
    } catch {
        Write-Host ('  cannot open {0}: {1}' -f $Port, $_.Exception.Message) -ForegroundColor Red
        exit 2
    }
}

$rc = 0
try {
    if (-not (Assert-Alive)) { $rc = 3 }
    else {
        if (($Test -eq 'battery') -or ($Test -eq 'all'))  { Test-Battery }
        if (($Test -eq 'symmetry') -or ($Test -eq 'all')) { Test-Symmetry }
        if ($Test -eq 'straight')                         { Test-Straight }
    }
} catch {
    Write-Host ('  ERROR: {0}' -f $_.Exception.Message) -ForegroundColor Red
    $rc = 4
} finally {
    # Whatever happened above, leave the motors requested-off before closing.
    # 'x' only clears the request; the PID task performs motor_stop() within one
    # 10 ms cycle (the motor keeps a single owner).
    if ($script:sp -and $script:sp.IsOpen) {
        try { $script:sp.Write('x'); Start-Sleep -Milliseconds 120 } catch { }
        try { $script:sp.Close(); $script:sp.Dispose() } catch { }
    }
    $body = "# bench_p0 $Test @ $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')`r`n" +
            "# full console capture, unfiltered`r`n`r`n" +
            (($script:log | ForEach-Object { $_.ToString() }) -join "`r`n") + "`r`n"
    [System.IO.File]::WriteAllText($logPath, $body, (New-Object System.Text.UTF8Encoding($false)))
    Write-Host ''
    Write-Host ('log written -> {0}' -f $logPath) -ForegroundColor DarkGray
}

exit $rc
