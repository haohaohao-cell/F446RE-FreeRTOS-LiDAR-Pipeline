@echo off
REM  Demo 1: zero the pose, then drive 1400 mm straight ahead. Double-click me.
setlocal
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -Command "& '%~dp0..\tools\bt_run.ps1' '0 1400'"
echo.
echo   The car stops by itself after about 13 s. Do not touch it.
echo   Then measure the axle centre against the 1.40 m mark.
pause
