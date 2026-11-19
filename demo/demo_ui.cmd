@echo off
REM ---------------------------------------------------------------------------
REM  One window that owns COM10: pick a target, the car drives there, and the
REM  same window keeps showing link health and heading.
REM
REM  While this window is open it holds the port, so bt_run / preflight_link will
REM  report "already open elsewhere".  Close it, or just use it.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\tools\drive_ui.ps1" %*
if errorlevel 1 pause
