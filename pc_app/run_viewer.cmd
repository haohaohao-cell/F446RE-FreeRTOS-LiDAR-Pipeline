@echo off
REM ---------------------------------------------------------------------------
REM  Launch the LiDAR point-cloud / IMU viewer (pc_app\main.py).
REM
REM    run_viewer.cmd            -> default port (usually COM10 = HC-04) @ 115200
REM    run_viewer.cmd COM7       -> explicit port
REM    run_viewer.cmd -l         -> list the available COM ports and exit
REM    run_viewer.cmd --sim      -> synthetic data, no hardware at all
REM
REM  Notes
REM   * The firmware sends the binary telemetry on UART5 at 230400, but Windows
REM     caps a Bluetooth SPP virtual COM port at 115200. That setting is only
REM     nominal for SPP - the radio does the real work - so 115200 is correct
REM     here and does NOT throttle anything.
REM   * Find the right port with -l, or in Windows Bluetooth settings look for
REM     the "Outgoing" port of the paired HC-04 module.
REM   * Needs Python 3 with matplotlib + pyserial + tkinter:
REM         py -3 -m pip install matplotlib pyserial
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"

REM --- pick a Python interpreter that actually has the dependencies --------
set PY=
py -3 -c "import matplotlib, serial" >nul 2>nul && set PY=py -3
if not defined PY (
    python -c "import matplotlib, serial" >nul 2>nul && set PY=python
)
if not defined PY (
    echo [ERROR] no Python with matplotlib + pyserial was found.
    echo         Install Python 3, then run:
    echo             py -3 -m pip install matplotlib pyserial
    pause
    exit /b 1
)

if /i "%~1"=="-l" (
    %PY% main.py
    pause
    exit /b 0
)

if /i "%~1"=="--sim" (
    echo Starting in SIMULATION mode - no hardware needed.
    %PY% main.py --sim
    exit /b 0
)

set PORT=%~1
if "%PORT%"=="" set PORT=COM10

echo Opening %PORT% @ 115200 (Bluetooth SPP) with: %PY%
%PY% main.py --port %PORT% --baud 115200

if errorlevel 1 (
    echo.
    echo [ERROR] could not open %PORT%.
    echo   - is the car powered on?  ^(the HC-04 only answers when powered^)
    echo   - is HC-04 paired in Windows Bluetooth settings?
    echo   - run:  run_viewer.cmd -l     to see which ports exist
    pause
)
