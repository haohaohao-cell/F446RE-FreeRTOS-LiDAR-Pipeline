@echo off
REM ---------------------------------------------------------------------------
REM  Role 6 evidence: send "G x y" over Bluetooth, wait for the firmware's own
REM  arrival counters, read the pose back, print the error table.
REM
REM    nav_run.cmd                 the three acceptance targets
REM    nav_run.cmd -Only 0,1400    just one
REM    nav_run.cmd -Simulate       no hardware; proves the logic only
REM    nav_run.cmd -SelfTest       parser check only
REM
REM  Needs BOTH links: COM8 (ST-Link VCP, 115200) and COM10 (HC-04 SPP, 115200).
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "nav_run.ps1" %*
