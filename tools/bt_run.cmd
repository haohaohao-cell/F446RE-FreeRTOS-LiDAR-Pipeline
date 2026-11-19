@echo off
REM ---------------------------------------------------------------------------
REM  ONE command for the demo: zero the pose, then drive to a target.
REM
REM    bt_run.cmd "0 1400"      straight ahead 1.40 m
REM    bt_run.cmd "-700 700"    straight 0.70 m, turn LEFT 90 deg, 0.70 m
REM    bt_run.cmd "700 700"     straight 0.70 m, turn RIGHT 90 deg, 0.70 m
REM
REM  -Command (not -File) is deliberate: with -File a target that starts with a
REM  minus sign ("-700 700") is parsed as a parameter name and the script never
REM  runs.  Inside -Command the quoted string stays a literal argument.
REM
REM  %* goes INSIDE single quotes for the same reason one level up: a caller that
REM  has already lost its own quoting (e.g. this script driven from another
REM  PowerShell as "0 300") otherwise hands PowerShell two positional arguments
REM  and it dies with "A positional parameter cannot be found".
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -Command "& '%~dp0bt_run.ps1' '%*'"
if errorlevel 1 pause
