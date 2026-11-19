@echo off
REM  Preflight: is the Bluetooth SPP link alive?  Double-click BEFORE the demo.
REM  The car must be powered on - the bytes being counted come from the car.
setlocal
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\tools\bt_link_check.ps1" -Sec 3
echo.
echo   Healthy = about 7000 B/s.  Zero bytes = the car will not hear anything.
pause
