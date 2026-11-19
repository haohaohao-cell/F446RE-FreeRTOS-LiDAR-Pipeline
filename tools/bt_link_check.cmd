@echo off
REM ---------------------------------------------------------------------------
REM  Is the Bluetooth SPP link alive?  Run this before placing the car on tape.
REM
REM    bt_link_check.cmd              3 s sample
REM    bt_link_check.cmd -Loop 30     watch for 30 s, drops show up as gaps
REM
REM  The car must be powered on: the bytes being counted are the firmware's own
REM  10 Hz telemetry stream, so a silent link means the car will not hear the
REM  next command either.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "%~dp0bt_link_check.ps1" %*
if errorlevel 1 pause
