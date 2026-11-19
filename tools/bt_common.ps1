# bt_common.ps1
#
# Shared Bluetooth-SPP helpers for the demo scripts.  Dot-source it:
#
#     . "$PSScriptRoot\bt_common.ps1"
#
# WHY THIS EXISTS
# ---------------
# The HC-04 link is the least reliable part of the demo, and the failure is
# silent in a nasty way: opening COM10 can succeed while the radio link is
# actually down, so the old scripts sent Z and G into a dead port and the car
# simply never moved.  The operator then blames the firmware.
#
# Two defences, both of which work WITHOUT any firmware change:
#
#   1. Retry the open.  A freshly woken SPP port refuses the first open while
#      the adapter re-establishes the link; 2-6 s of retrying is usually enough
#      and costs nothing when the port is already fine.
#
#   2. Probe the link by READING, not by trusting Open().  The firmware streams
#      the 10 Hz telemetry frame over UART5 continuously (telemetry_send_data()
#      in the btTx task), so a live link delivers thousands of bytes per second
#      even with nothing else connected.  Bytes arriving is proof that the radio
#      link, the module's UART and the PC's adapter are all alive end to end.
#      No bytes means "do not send the drive command yet".
#
# The byte rate is deliberately reported instead of just "OK": a link that is
# fading (a few hundred bytes/s instead of ~5 kB/s) is exactly what a dying
# cable or a battery-brownout module looks like, and it is worth seeing before
# the car is placed on the tape.

$script:BtBaud = 115200

function Get-BtPortList
{
    return @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
}

function Open-BtPort
{
    param(
        [string]$Port,
        [int]$Attempts = 10,
        [int]$DelayMs = 600
    )

    $missingReported = $false

    for ($i = 1; $i -le $Attempts; $i++)
    {
        # The port can disappear while the adapter sleeps and come back a
        # second later, so "not in GetPortNames()" is a hint, not a verdict:
        # keep retrying and report the list once so the operator can see it.
        if (-not $missingReported -and ((Get-BtPortList) -notcontains $Port))
        {
            $missingReported = $true
            Write-Host ("port {0} is not listed by Windows..." -f $Port) -ForegroundColor Yellow
            Write-Host ("  ports visible now: {0}" -f ((Get-BtPortList) -join ' ')) -ForegroundColor DarkGray
        }

        try
        {
            $sp = New-Object System.IO.Ports.SerialPort($Port, $script:BtBaud, 'None', 8, 'One')
            $sp.ReadTimeout = 100
            $sp.WriteTimeout = 2000
            $sp.Open()
            if ($i -gt 1)
            {
                Write-Host ("port {0} opened on attempt {1}" -f $Port, $i) -ForegroundColor DarkGray
            }
            return $sp
        }
        catch
        {
            $msg = $_.Exception.Message
            # "Access to the port is denied" is NOT transient: the point-cloud
            # viewer (or a previous run) still owns the handle.  Retrying ten
            # times would just burn six seconds on stage.
            if ($msg -match 'denied|being used|in use|refused|not exist')
            {
                Write-Host ("cannot open {0}: the port is already open elsewhere" -f $Port) -ForegroundColor Red
                Write-Host '  close the PC point-cloud viewer (pc_app) and try again.' -ForegroundColor Yellow
                Write-Host '  a killed bt_run window can also hold it for a second - wait and retry.' -ForegroundColor DarkGray
                return $null
            }
            if ($i -eq $Attempts)
            {
                Write-Host ("cannot open {0} after {1} attempts: {2}" -f $Port, $Attempts, $msg) -ForegroundColor Red
                # A "semaphore timeout" on Open() is the SPP link failing to come
                # up: the virtual port exists (it is paired) but the radio is
                # not talking.  That is a different problem from a busy port and
                # needs battery/module/pairing, not a retry.
                if ($msg -match 'semaphore|timeout|超时')
                {
                    Write-Host '  that error means the radio link is down, not that the port is busy:' -ForegroundColor Yellow
                    Write-Host '  - is the car powered on? (the module is powered from the car)' -ForegroundColor Yellow
                    Write-Host '  - module LED: steady = connected, blinking = no link' -ForegroundColor Yellow
                    Write-Host '  - battery charged? a brown-out resets the module mid-run' -ForegroundColor Yellow
                    Write-Host '  - Windows: Device Manager > Bluetooth adapter > Power Management' -ForegroundColor DarkGray
                }
                Write-Host '  - HC-04 powered and paired?' -ForegroundColor Yellow
                Write-Host '  - is the PC point-cloud viewer holding the port?' -ForegroundColor Yellow
                return $null
            }
            Start-Sleep -Milliseconds $DelayMs
        }
    }
    return $null
}

function Measure-BtLink
{
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$Ms = 500
    )

    $buf = New-Object byte[] 4096
    $bytes = 0
    try { $Port.DiscardInBuffer() } catch { }

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $Ms)
    {
        try
        {
            $n = $Port.Read($buf, 0, $buf.Length)
            if ($n -gt 0) { $bytes += $n }
        }
        catch [System.TimeoutException]
        {
            # nothing in this 100 ms slice - keep waiting out the window
        }
        catch
        {
            break
        }
    }
    $sw.Stop()

    $rate = 0
    if ($sw.ElapsedMilliseconds -gt 0)
    {
        $rate = [int]($bytes * 1000 / $sw.ElapsedMilliseconds)
    }

    return [pscustomobject]@{
        Bytes = $bytes
        Ms    = [int]$sw.ElapsedMilliseconds
        Rate  = $rate
    }
}

function Write-BtLinkReport
{
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$Ms = 500
    )

    $m = Measure-BtLink -Port $Port -Ms $Ms
    # The firmware streams ~5 kB/s (20 frames/s of ~256 B).  Anything above
    # ~1 kB/s means the radio link is genuinely carrying the stream; a few
    # hundred bytes means it is up but dying; zero means it is down.
    if ($m.Rate -ge 1000)
    {
        Write-Host ("link: OK, {0} B/s of telemetry ({1} B in {2} ms)" -f $m.Rate, $m.Bytes, $m.Ms) -ForegroundColor Green
        return $true
    }
    elseif ($m.Bytes -gt 0)
    {
        Write-Host ("link: WEAK, only {0} B/s ({1} B in {2} ms) - expected ~5000 B/s" -f $m.Rate, $m.Bytes, $m.Ms) -ForegroundColor Yellow
        Write-Host '  the radio link is up but dropping data. If the car misbehaves, re-run.' -ForegroundColor Yellow
        return $true
    }

    Write-Host ("link: NO DATA in {0} ms - the SPP link is down even though the port opened" -f $m.Ms) -ForegroundColor Red
    Write-Host '  the car will NOT receive the command. Do not rely on this run.' -ForegroundColor Yellow
    Write-Host '  - wait 3 s and double-click again (the adapter usually recovers)' -ForegroundColor DarkGray
    Write-Host '  - check the module LED: steady = connected, blinking = no link' -ForegroundColor DarkGray
    return $false
}

function Send-BtLine
{
    param(
        [System.IO.Ports.SerialPort]$Port,
        [string]$Line,
        [int]$SettleMs = 400
    )

    try
    {
        $Port.Write($Line + "`n")
        # The SPP buffer must drain before the handle closes, otherwise the last
        # command can arrive late (or not at all) - the "late STOP" bug.
        Start-Sleep -Milliseconds $SettleMs
        return $true
    }
    catch
    {
        Write-Host ("write failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
        return $false
    }
}
