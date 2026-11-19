@echo off
REM  Demo 3: zero the pose, then drive to (700, -700) - a 135 deg right pivot first.
setlocal
cd /d "%~dp0..\tools"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "bt_run.ps1" "700 -700"
echo.
echo   Carry the car back to the start mark BEFORE running this one.
pause
