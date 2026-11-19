@echo off
REM  Demo 2: zero the pose, then drive 700 mm straight, turn LEFT 90 deg,
REM  then 700 mm. Target (-700, 700) - the left-hand arm of the cross maze.
REM  Double-click me AFTER carrying the car back to the start mark.
setlocal
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -Command "& '%~dp0..\tools\bt_run.ps1' '-700 700'"
echo.
echo   Carry the car back to the start mark BEFORE running this one.
pause
