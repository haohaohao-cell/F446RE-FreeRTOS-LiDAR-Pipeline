@echo off
REM ---------------------------------------------------------------------------
REM  Live debug console for USART2 (ST-Link Virtual COM Port).
REM
REM    serial_console.cmd                -> COM8 @ 115200, run until Ctrl+C
REM    serial_console.cmd -Port COM4     -> another port
REM    serial_console.cmd -Seconds 5     -> run 5 s then stop (self-test)
REM
REM  This is where blk / idle / q1 / q2 / latmax / parsemax / heap / stk live.
REM  The point-cloud window (pc_app\run_viewer.cmd) is a DIFFERENT link:
REM  UART5 -> Bluetooth -> COM10, binary frames only.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "serial_console.ps1" %*
