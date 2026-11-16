@echo off
REM ---------------------------------------------------------------------------
REM  Wrapper so the helper runs even when the machine's PowerShell execution
REM  policy is Restricted (the .ps1 alone would be refused).
REM
REM  Usage:
REM      flash_and_monitor.cmd -Probe
REM      flash_and_monitor.cmd
REM      flash_and_monitor.cmd -NoFlash -Seconds 30
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "flash_and_monitor.ps1" %*
exit /b %ERRORLEVEL%
