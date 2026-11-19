@echo off
REM ---------------------------------------------------------------------------
REM  ONE command for the demo: zero the pose, then drive to a target.
REM
REM    bt_run.cmd "0 1400"      straight ahead 1.4 m
REM    bt_run.cmd "-700 700"    left-forward 45 deg   (keep the quotes)
REM    bt_run.cmd "700 -700"    right-rear 135 deg
REM
REM  -Command (not -File) is deliberate: with -File a target that starts with a
REM  minus sign ("-700 700") is parsed as a parameter name and the script never
REM  runs.  Inside -Command the quoted string stays a literal argument.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -Command "& '%~dp0bt_run.ps1' %*"
if errorlevel 1 pause
