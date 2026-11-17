@echo off
setlocal
rem ---------------------------------------------------------------------------
rem Host-side unit tests for the pure logic in the Part 2 firmware.
rem
rem These need ANY host C compiler (the firmware toolchain cannot run on the PC,
rem so arm-none-eabi-gcc is useless here).  MinGW-w64 gcc is used on this machine.
rem
rem What is covered, and what is NOT:
rem   covered : bt_parse_line / bt_error_text   (protocol grammar + bounds)
rem             path_generate_line / path_track_step / path_arrived (geometry)
rem             pose_update sign conventions               (odometry)
rem   NOT     : anything that touches the HAL, DMA, the RTOS or the real panel.
rem             Those are hardware items - see Docs\HANDOVER.md 7.1.
rem ---------------------------------------------------------------------------
cd /d "%~dp0"

set CC=gcc
where %CC% >nul 2>nul
if errorlevel 1 (
  echo [ERROR] no host compiler on PATH. Install MinGW-w64 or edit CC= in this script.
  exit /b 1
)

set INC=-I hostinc -I ..\..\Core\Inc
set FLAGS=-std=gnu11 -Wall -Wextra -O1
set FAILED=0

echo === test_bt_protocol ===
%CC% %FLAGS% %INC% test_bt_protocol.c ..\..\Core\Src\bt_protocol.c -o test_bt_protocol.exe || goto :fail
test_bt_protocol.exe || set FAILED=1

echo === test_path ===
%CC% %FLAGS% %INC% test_path.c ..\..\Core\Src\path.c -o test_path.exe -lm || goto :fail
test_path.exe || set FAILED=1

echo === test_pose ===
%CC% %FLAGS% %INC% test_pose.c ..\..\Core\Src\pose.c -o test_pose.exe -lm || goto :fail
test_pose.exe || set FAILED=1

if "%FAILED%"=="1" (
  echo.
  echo SOME TESTS FAILED
  exit /b 1
)
echo.
echo ALL HOST TESTS PASSED
exit /b 0

:fail
echo.
echo BUILD FAILED
exit /b 1
