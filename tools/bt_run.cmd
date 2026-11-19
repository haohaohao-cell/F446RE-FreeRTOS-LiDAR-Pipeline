@echo off
REM ---------------------------------------------------------------------------
REM  ONE command for the demo: zero the pose, then drive to a target.
REM
REM    bt_run.cmd "0 1400"      straight ahead 1.4 m
REM    bt_run.cmd "-700 700"    left-forward 45 deg   (keep the quotes)
REM    bt_run.cmd "700 -700"    right-rear 135 deg
REM
REM  Nothing but Bluetooth is touched: no USB cable, no COM8.
REM  Between runs: carry the car back to the start mark, then run this again.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "bt_run.ps1" %*
