@echo off
REM ---------------------------------------------------------------------------
REM  P0 bench measurements, driven from the PC over USART2 (ST-Link VCP).
REM
REM    bench_p0.cmd -Test battery     P0 #1  pack health at 20 % duty (wheels up)
REM    bench_p0.cmd -Test symmetry    P0 #3  right wheel counts vs left
REM    bench_p0.cmd -Test straight    P0 #2  open-loop line, drift per metre
REM    bench_p0.cmd -Test all
REM    bench_p0.cmd -SelfTest         parser check only, no board needed
REM
REM  Adds no firmware code: it presses the console keys that already exist
REM  (c f b l r x e) with the spacing that console_poll() needs.
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"
set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -ExecutionPolicy Bypass -File "bench_p0.ps1" %*
