@echo off
REM ---------------------------------------------------------------------------
REM  Send ONE command over Bluetooth, no USB serial needed.
REM
REM    bt_send.cmd "G 0 1400"     drive to (0, 1400) mm   (x right, y forward)
REM    bt_send.cmd "G -700 700"   negative targets: keep the quotes
REM    bt_send.cmd "S"            stop and clear the path
REM
REM  This is the acceptance path: the car runs untethered, the endpoint is
REM  measured with a tape.  nav_run.cmd is the bench tool (it also needs COM8).
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "bt_send.ps1" %*
