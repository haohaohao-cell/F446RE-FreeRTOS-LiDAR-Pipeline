# drive_ui.ps1
#
# One window that owns COM10 for the whole demo: type a coordinate, the car
# drives there; the same window keeps reading the telemetry stream, so the link
# health and the heading are visible the entire time.
#
# WHY A LONG-LIVED PORT
# ---------------------
# Every previous demo step opened COM10, sent, and closed.  Each open can fail
# (the adapter has to bring the RFCOMM link up) and each close leaves a window in
# which Windows may drop it again.  Holding the port open removes both, and it
# also makes the link state observable instead of guessed: the byte rate in this
# window is the same evidence bt_link_check.cmd prints.
#
# WHAT IT CAN AND CANNOT SHOW
# ---------------------------
# The telemetry frame carries heading and the LiDAR scan, NOT the pose x/y
# (telemetry.c: heading, gz, ax, ay, point count, distances).  So the window can
# show where the car is pointing and what it sees, but not where it thinks it is.
# That number only exists on COM8 (USART2, wired), which is why the tape measure
# is still the acceptance evidence.
#
# USAGE
#   demo\demo_ui.cmd                 (double-click)
#   tools\drive_ui.ps1 -Port COM10
#
# While this window is open it owns COM10: bt_run.cmd / bt_send.cmd /
# preflight_link.cmd will report "already open elsewhere".  Close it first, or
# just use it.
[CmdletBinding()]
param(
    [string]$Port = 'COM10'
)

$ErrorActionPreference = 'Stop'

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()

# ---------------------------------------------------------------------------
# state
# ---------------------------------------------------------------------------
$script:sp            = $null
$script:rx            = New-Object 'System.Collections.Generic.List[byte]'
$script:okFrames      = 0
$script:synced        = $false
$script:badFrames     = 0
$script:framesTick    = 0
$script:fps           = 0
$script:bytesTick     = 0
$script:rate          = 0
$script:lastTick      = [DateTime]::Now
$script:heading       = $null
$script:minDist       = $null
$script:lastSend      = ''

$font       = New-Object System.Drawing.Font('Microsoft YaHei UI', 10)
$fontSmall  = New-Object System.Drawing.Font('Microsoft YaHei UI', 9)
$fontBig    = New-Object System.Drawing.Font('Microsoft YaHei UI', 20, [System.Drawing.FontStyle]::Bold)
$green      = [System.Drawing.Color]::FromArgb(0, 128, 0)
$red        = [System.Drawing.Color]::FromArgb(192, 0, 0)
$amber      = [System.Drawing.Color]::FromArgb(190, 130, 0)
$grey       = [System.Drawing.Color]::FromArgb(90, 90, 90)

# ---------------------------------------------------------------------------
# form
# ---------------------------------------------------------------------------
$form                 = New-Object System.Windows.Forms.Form
$form.Text            = '循迹小车 · 蓝牙遥控台（Block 4.5 Part 2）'
$form.AutoScaleMode   = 'None'
$form.ClientSize      = New-Object System.Drawing.Size(796, 662)
$form.StartPosition   = 'CenterScreen'
$form.FormBorderStyle = 'FixedSingle'
$form.MaximizeBox     = $false
$form.Font            = $font
$form.BackColor       = [System.Drawing.Color]::White

# --- link row ---------------------------------------------------------------
$grpLink            = New-Object System.Windows.Forms.GroupBox
$grpLink.Text       = '蓝牙链路'
$grpLink.Location   = New-Object System.Drawing.Point(12, 10)
$grpLink.Size       = New-Object System.Drawing.Size(772, 96)
$grpLink.Font       = $fontSmall

$lblPort            = New-Object System.Windows.Forms.Label
$lblPort.Text       = '端口'
$lblPort.Location   = New-Object System.Drawing.Point(14, 28)
$lblPort.Size       = New-Object System.Drawing.Size(40, 24)

$txtPort            = New-Object System.Windows.Forms.TextBox
$txtPort.Text       = $Port
$txtPort.Location   = New-Object System.Drawing.Point(56, 25)
$txtPort.Size       = New-Object System.Drawing.Size(80, 26)

$btnConnect         = New-Object System.Windows.Forms.Button
$btnConnect.Text    = '连接'
$btnConnect.Location = New-Object System.Drawing.Point(146, 23)
$btnConnect.Size    = New-Object System.Drawing.Size(84, 30)

$lblLink            = New-Object System.Windows.Forms.Label
$lblLink.Text       = '未连接'
$lblLink.Location   = New-Object System.Drawing.Point(246, 26)
$lblLink.Size       = New-Object System.Drawing.Size(512, 26)
$lblLink.Font       = New-Object System.Drawing.Font('Microsoft YaHei UI', 11, [System.Drawing.FontStyle]::Bold)
$lblLink.ForeColor  = $grey

$lblRate            = New-Object System.Windows.Forms.Label
$lblRate.Text       = '字节率 —   ·   帧率 —   ·   好帧 0 / 坏帧 0'
$lblRate.Location   = New-Object System.Drawing.Point(14, 62)
$lblRate.Size       = New-Object System.Drawing.Size(744, 24)
$lblRate.ForeColor  = $grey

$grpLink.Controls.AddRange(@($lblPort, $txtPort, $btnConnect, $lblLink, $lblRate))

# --- live row ---------------------------------------------------------------
$grpLive            = New-Object System.Windows.Forms.GroupBox
$grpLive.Text       = '实时（来自车上遥测）'
$grpLive.Location   = New-Object System.Drawing.Point(12, 112)
$grpLive.Size       = New-Object System.Drawing.Size(772, 118)
$grpLive.Font       = $fontSmall

$lblHeadCap         = New-Object System.Windows.Forms.Label
$lblHeadCap.Text    = '航向'
$lblHeadCap.Location = New-Object System.Drawing.Point(14, 26)
$lblHeadCap.Size    = New-Object System.Drawing.Size(60, 24)

$lblHead            = New-Object System.Windows.Forms.Label
$lblHead.Text       = '—'
$lblHead.Location   = New-Object System.Drawing.Point(70, 18)
$lblHead.Size       = New-Object System.Drawing.Size(220, 40)
$lblHead.Font       = $fontBig

$lblHeadHint        = New-Object System.Windows.Forms.Label
$lblHeadHint.Text   = "0° = 车头朝向铺设方向；左转为正、右转为负"
$lblHeadHint.Location = New-Object System.Drawing.Point(16, 64)
$lblHeadHint.Size   = New-Object System.Drawing.Size(360, 24)
$lblHeadHint.ForeColor = $grey

$lblDistCap         = New-Object System.Windows.Forms.Label
$lblDistCap.Text    = '最近回波'
$lblDistCap.Location = New-Object System.Drawing.Point(400, 26)
$lblDistCap.Size    = New-Object System.Drawing.Size(80, 24)

$lblDist            = New-Object System.Windows.Forms.Label
$lblDist.Text       = '—'
$lblDist.Location   = New-Object System.Drawing.Point(486, 18)
$lblDist.Size       = New-Object System.Drawing.Size(262, 40)
$lblDist.Font       = $fontBig

$lblDistHint        = New-Object System.Windows.Forms.Label
$lblDistHint.Text   = '激光雷达 360 点里的最小值'
$lblDistHint.Location = New-Object System.Drawing.Point(402, 64)
$lblDistHint.Size   = New-Object System.Drawing.Size(320, 24)
$lblDistHint.ForeColor = $grey

$grpLive.Controls.AddRange(@($lblHeadCap, $lblHead, $lblHeadHint, $lblDistCap, $lblDist, $lblDistHint))

# --- command row ------------------------------------------------------------
$grpCmd             = New-Object System.Windows.Forms.GroupBox
$grpCmd.Text        = '发命令（点一下 = 清零位姿 + 走）'
$grpCmd.Location    = New-Object System.Drawing.Point(12, 236)
$grpCmd.Size        = New-Object System.Drawing.Size(772, 140)
$grpCmd.Font        = $fontSmall

$btnStraight        = New-Object System.Windows.Forms.Button
$btnStraight.Text   = "直行 1.40 m`n(0, 1400)"
$btnStraight.Location = New-Object System.Drawing.Point(14, 24)
$btnStraight.Size   = New-Object System.Drawing.Size(168, 50)
$btnStraight.Font   = $font

$btnLeft            = New-Object System.Windows.Forms.Button
$btnLeft.Text       = "左臂（先直行再左转）`n(−700, 700)"
$btnLeft.Location   = New-Object System.Drawing.Point(190, 24)
$btnLeft.Size       = New-Object System.Drawing.Size(168, 50)
$btnLeft.Font       = $font

$btnRight           = New-Object System.Windows.Forms.Button
$btnRight.Text      = "右臂（先直行再右转）`n(700, 700)"
$btnRight.Location  = New-Object System.Drawing.Point(366, 24)
$btnRight.Size      = New-Object System.Drawing.Size(168, 50)
$btnRight.Font      = $font

$btnStop            = New-Object System.Windows.Forms.Button
$btnStop.Text       = "急停`n(S)"
$btnStop.Location   = New-Object System.Drawing.Point(556, 24)
$btnStop.Size       = New-Object System.Drawing.Size(200, 50)
$btnStop.Font       = New-Object System.Drawing.Font('Microsoft YaHei UI', 11, [System.Drawing.FontStyle]::Bold)
$btnStop.BackColor  = [System.Drawing.Color]::FromArgb(255, 235, 235)

$lblX               = New-Object System.Windows.Forms.Label
$lblX.Text          = 'x ='
$lblX.Location      = New-Object System.Drawing.Point(16, 92)
$lblX.Size          = New-Object System.Drawing.Size(28, 24)

$txtX               = New-Object System.Windows.Forms.TextBox
$txtX.Text          = '0'
$txtX.Location      = New-Object System.Drawing.Point(44, 89)
$txtX.Size          = New-Object System.Drawing.Size(72, 26)

$lblY               = New-Object System.Windows.Forms.Label
$lblY.Text          = 'y ='
$lblY.Location      = New-Object System.Drawing.Point(126, 92)
$lblY.Size          = New-Object System.Drawing.Size(28, 24)

$txtY               = New-Object System.Windows.Forms.TextBox
$txtY.Text          = '300'
$txtY.Location      = New-Object System.Drawing.Point(154, 89)
$txtY.Size          = New-Object System.Drawing.Size(72, 26)

$btnCustom          = New-Object System.Windows.Forms.Button
$btnCustom.Text     = '发送 G x y'
$btnCustom.Location = New-Object System.Drawing.Point(238, 86)
$btnCustom.Size     = New-Object System.Drawing.Size(120, 32)

$lblUnit            = New-Object System.Windows.Forms.Label
$lblUnit.Text       = 'mm，+x = 车右，+y = 车头向前，范围 ±5000'
$lblUnit.Location   = New-Object System.Drawing.Point(370, 92)
$lblUnit.Size       = New-Object System.Drawing.Size(392, 24)
$lblUnit.ForeColor  = $grey

$grpCmd.Controls.AddRange(@($btnStraight, $btnLeft, $btnRight, $btnStop,
                            $lblX, $txtX, $lblY, $txtY, $btnCustom, $lblUnit))

# --- log --------------------------------------------------------------------
$lblLogCap          = New-Object System.Windows.Forms.Label
$lblLogCap.Text     = '日志'
$lblLogCap.Location = New-Object System.Drawing.Point(14, 384)
$lblLogCap.Size     = New-Object System.Drawing.Size(60, 22)
$lblLogCap.ForeColor = $grey

$txtLog             = New-Object System.Windows.Forms.TextBox
$txtLog.Multiline   = $true
$txtLog.ReadOnly    = $true
$txtLog.ScrollBars  = 'Vertical'
$txtLog.Location    = New-Object System.Drawing.Point(14, 408)
$txtLog.Size        = New-Object System.Drawing.Size(770, 196)
$txtLog.Font        = New-Object System.Drawing.Font('Consolas', 9)
$txtLog.BackColor   = [System.Drawing.Color]::FromArgb(250, 250, 250)

$form.Controls.AddRange(@($grpLink, $grpLive, $grpCmd, $lblLogCap, $txtLog))

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
function Write-Log
{
    param([string]$Message, [string]$Level = 'info')

    $stamp = (Get-Date).ToString('HH:mm:ss')
    $txtLog.AppendText(("[{0}] {1}`r`n" -f $stamp, $Message)) | Out-Null
    $txtLog.SelectionStart = $txtLog.TextLength
    $txtLog.ScrollToCaret()
}

# Blocking waits must keep the window alive, otherwise the UI freezes for a
# second on every send and Windows paints it as "not responding".
function Wait-Pump
{
    param([int]$Milliseconds)
    $end = (Get-Date).AddMilliseconds($Milliseconds)
    while ((Get-Date) -lt $end)
    {
        [System.Windows.Forms.Application]::DoEvents()
        Start-Sleep -Milliseconds 10
    }
}

function Test-Port
{
    return (($null -ne $script:sp) -and $script:sp.IsOpen)
}

function Set-LinkLabel
{
    param([string]$Text, [System.Drawing.Color]$Colour)
    $lblLink.Text = $Text
    $lblLink.ForeColor = $Colour
}

function Close-Port
{
    if ($null -ne $script:sp)
    {
        try { $script:sp.Close() } catch { }
        try { $script:sp.Dispose() } catch { }
        $script:sp = $null
    }
    $script:rx.Clear()
    Set-LinkLabel '未连接' $grey
    $btnConnect.Text = '连接'
}

function Open-Port
{
    $name = $txtPort.Text.Trim()
    if ($name -eq '') { Write-Log '端口名是空的'; return }

    $visible = @([System.IO.Ports.SerialPort]::GetPortNames())
    if ($visible -notcontains $name)
    {
        Write-Log ("端口 {0} 不在列表里。Windows 现在看得见：{1}" -f $name, ($visible -join ' ')) 'err'
        Write-Log '  蓝牙没配对 / 车没上电 / 端口号变了' 'err'
        Set-LinkLabel '端口不存在' $red
        return
    }

    Write-Log ("正在打开 {0} ..." -f $name)
    for ($i = 1; $i -le 8; $i++)
    {
        try
        {
            $sp = New-Object System.IO.Ports.SerialPort($name, 115200, 'None', 8, 'One')
            $sp.ReadTimeout = 100
            $sp.WriteTimeout = 2000
            $sp.Open()
            $script:sp = $sp
            $script:rx.Clear()
            $script:okFrames = 0
            $script:badFrames = 0
            $script:synced = $false
            $script:lastTick = [DateTime]::Now
            $script:bytesTick = 0
            $script:framesTick = 0
            $btnConnect.Text = '断开'
            Set-LinkLabel '已打开，等待遥测…' $amber
            Write-Log ("{0} 已打开（第 {1} 次尝试）" -f $name, $i) 'ok'
            return
        }
        catch
        {
            $msg = $_.Exception.Message
            if ($msg -match 'denied|being used|in use')
            {
                Write-Log ("打不开 {0}：端口被别的程序占着" -f $name) 'err'
                Write-Log '  关掉点云窗口 / 另一个 bt_run 窗口，再点连接' 'err'
                Set-LinkLabel '端口被占用' $red
                return
            }
            if ($i -eq 8)
            {
                Write-Log ("打不开 {0}：{1}" -f $name, $msg) 'err'
                if ($msg -match 'semaphore|timeout')
                {
                    Write-Log '  这是无线链路没起来（不是端口被占）：车没上电 / 没配对 / 电池弱' 'err'
                }
                Set-LinkLabel '打开失败' $red
                return
            }
            Wait-Pump 600
        }
    }
}

function Send-Line
{
    param([string]$Line, [switch]$Twice)

    if (-not (Test-Port))
    {
        Write-Log '串口没打开，先点"连接"' 'err'
        return $false
    }
    try
    {
        $script:sp.Write($Line + "`n")
        if ($Twice)
        {
            Wait-Pump 150
            $script:sp.Write($Line + "`n")
        }
        return $true
    }
    catch
    {
        Write-Log ("写串口失败：{0}" -f $_.Exception.Message) 'err'
        Set-LinkLabel '写入失败，链路可能断了' $red
        return $false
    }
}

function Send-Target
{
    param([int]$X, [int]$Y)

    if (-not (Test-Port)) { Write-Log '串口没打开，先点"连接"' 'err'; return }
    if ($X -lt -5000 -or $X -gt 5000 -or $Y -lt -5000 -or $Y -gt 5000)
    {
        Write-Log ("目标 ({0}, {1}) 超出 ±5000 mm" -f $X, $Y) 'err'
        return
    }

    # Z twice: the first write after an SPP connection comes up is sometimes
    # swallowed, and a lost Z means G runs against the previous run's pose.
    if (-not (Send-Line 'Z')) { return }
    Wait-Pump 250
    if (-not (Send-Line 'Z')) { return }
    Wait-Pump 700

    if (-not (Send-Line ('G {0} {1}' -f $X, $Y))) { return }

    $legs = [Math]::Abs($X) + [Math]::Abs($Y)
    $secs = $legs / 120.0
    $script:lastSend = ('G {0} {1}' -f $X, $Y)
    Write-Log ("已发送 Z ×2 + G {0} {1}" -f $X, $Y) 'ok'
    Write-Log ("     轴对齐路径 {0} mm（直行 {1} mm → 90° → 横移 {2} mm），约 {3:F0} s" -f `
               $legs, [Math]::Abs($Y), [Math]::Abs($X), $secs)
    Write-Log '     车会自己停；别用手碰，等它停稳再量'
}

function Read-Telemetry
{
    if (-not (Test-Port)) { return }
    try
    {
        $available = $script:sp.BytesToRead
        if ($available -gt 0)
        {
            $tmp = New-Object byte[] $available
            $got = $script:sp.Read($tmp, 0, $available)
            for ($i = 0; $i -lt $got; $i++) { $script:rx.Add($tmp[$i]) }
            $script:bytesTick += $got
        }
    }
    catch
    {
        Write-Log ("读串口失败：{0}" -f $_.Exception.Message) 'err'
        Close-Port
        return
    }

    # frame: AA 55 <type> <lenLo> <lenHi> <payload...> <xor>
    while ($script:rx.Count -ge 6)
    {
        if (($script:rx[0] -ne 0xAA) -or ($script:rx[1] -ne 0x55))
        {
            $script:rx.RemoveAt(0)
            continue
        }
        $len   = [int]$script:rx[3] -bor ([int]$script:rx[4] -shl 8)
        $total = 5 + $len + 1
        if ($script:rx.Count -lt $total) { break }

        $calc = 0
        for ($i = 2; $i -lt (5 + $len); $i++) { $calc = $calc -bxor $script:rx[$i] }
        if ($calc -ne $script:rx[5 + $len])
        {
            if ($script:synced) { $script:badFrames++ }
            $script:rx.RemoveAt(0)
            continue
        }

        if (($script:rx[2] -eq 1) -and ($len -ge 12))
        {
            # First good frame: from here on a checksum failure is a real error,
            # not the half-frame the port was opened in the middle of.
            $script:synced = $true

            $raw = [int]$script:rx[5] -bor ([int]$script:rx[6] -shl 8)
            if ($raw -ge 32768) { $raw -= 65536 }
            $script:heading = $raw / 100.0

            $points = [int]$script:rx[13] -bor ([int]$script:rx[14] -shl 8)
            $minD = 65535
            for ($i = 0; $i -lt $points; $i++)
            {
                $d = [int]$script:rx[15 + (2 * $i)] -bor ([int]$script:rx[16 + (2 * $i)] -shl 8)
                if (($d -gt 0) -and ($d -lt $minD)) { $minD = $d }
            }
            if ($minD -ne 65535) { $script:minDist = $minD }

            $script:okFrames++
            $script:framesTick++
        }

        $script:rx.RemoveRange(0, $total)
    }
}

function Update-Readouts
{
    $now = [DateTime]::Now
    $elapsed = ($now - $script:lastTick).TotalMilliseconds
    if ($elapsed -ge 1000)
    {
        $script:rate   = [int]($script:bytesTick * 1000 / $elapsed)
        $script:fps    = [Math]::Round($script:framesTick * 1000 / $elapsed, 1)
        $script:bytesTick  = 0
        $script:framesTick = 0
        $script:lastTick   = $now

        $lblRate.Text = ("字节率 {0} B/s   ·   帧率 {1}/s   ·   好帧 {2} / 坏帧 {3}" -f `
                         $script:rate, $script:fps, $script:okFrames, $script:badFrames)

        if (Test-Port)
        {
            if ($script:rate -ge 1000)      { Set-LinkLabel ("链路正常 · {0} B/s" -f $script:rate) $green }
            elseif ($script:rate -gt 0)     { Set-LinkLabel ("链路很弱 · {0} B/s（可能丢命令）" -f $script:rate) $amber }
            else                            { Set-LinkLabel '链路无数据（车没在发遥测）' $red }
        }
    }

    if ($null -ne $script:heading)
    {
        $lblHead.Text = ('{0:F1}°' -f $script:heading)
        if ([Math]::Abs($script:heading) -lt 5.0) { $lblHead.ForeColor = $green }
        else { $lblHead.ForeColor = [System.Drawing.Color]::Black }
    }
    if ($null -ne $script:minDist)
    {
        $lblDist.Text = ('{0} mm' -f $script:minDist)
    }
}

# ---------------------------------------------------------------------------
# events
# ---------------------------------------------------------------------------
$btnConnect.Add_Click({
    if (Test-Port) { Close-Port; Write-Log '已断开' }
    else { Open-Port }
})

$btnStraight.Add_Click({ Send-Target -X 0 -Y 1400 })
$btnLeft.Add_Click({ Send-Target -X -700 -Y 700 })
$btnRight.Add_Click({ Send-Target -X 700 -Y 700 })
$btnStop.Add_Click({
    if (Send-Line 'S' -Twice) { Write-Log '已发送 S（急停，清路径）' 'ok' }
})

$btnCustom.Add_Click({
    $x = 0; $y = 0
    if (-not [int]::TryParse($txtX.Text.Trim(), [ref]$x) -or
        -not [int]::TryParse($txtY.Text.Trim(), [ref]$y))
    {
        Write-Log 'x / y 必须是整数（单位 mm）' 'err'
        return
    }
    Send-Target -X $x -Y $y
})

$timer          = New-Object System.Windows.Forms.Timer
$timer.Interval = 150
$timer.Add_Tick({
    Read-Telemetry
    Update-Readouts
})

$form.Add_Shown({
    Write-Log '窗口已就绪。点"连接"占用 COM10，然后点目标按钮或输入坐标。'
    Write-Log '注意：窗口开着时 bt_run / preflight_link 打不开 COM10（正常）。'
    Write-Log '航向来自遥测；车自己认为的 x/y 只在 COM8（USB 线）上有，这个窗口看不到。'
    if ($txtPort.Text.Trim() -ne '') { Open-Port }
})

$form.Add_FormClosing({
    $timer.Stop()
    Close-Port
})

$timer.Start()
[void]$form.ShowDialog()
