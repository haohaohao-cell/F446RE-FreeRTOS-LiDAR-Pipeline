@echo off
REM ---------------------------------------------------------------------------
REM  Show the two lines that matter, WITHOUT reflashing:
REM
REM      [drive] frame: REVERSED (drives wheels-first)   <- which build is on
REM      Block4.5 Part2 ready imu=0 oled=0 ...           <- sensor status
REM
REM  It opens COM8 first, then resets the target over SWD, so the banner cannot
REM  be missed.  Nothing is programmed or erased.
REM
REM  The car must be powered: USB only feeds the ST-Link, the board runs on the
REM  battery.  If the target voltage reads ~0.1 V, switch the car on.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "%~dp0banner.ps1" %*
echo.
pause
