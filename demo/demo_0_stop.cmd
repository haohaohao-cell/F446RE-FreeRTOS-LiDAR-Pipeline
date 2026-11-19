@echo off
REM  Emergency stop / clear the path. Safe to run at any time.
setlocal
cd /d "%~dp0..\tools"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "bt_send.ps1" "S"
echo.
pause
