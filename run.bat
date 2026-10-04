@echo off
setlocal
cd /d "%~dp0"
title Autonomous Drone Line Follower Simulator

echo ========================================================
echo   Launching Autonomous Drone Simulator (Stage 1)...
echo ========================================================

:: Check for the installed Python 3.11 executable directly
set "PY_EXE=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"

if exist "%PY_EXE%" (
    echo Using Python at: "%PY_EXE%"
    "%PY_EXE%" main.py
) else (
    echo Using system python...
    python main.py
)

if errorlevel 1 (
    echo.
    echo An error occurred while running the simulator.
    pause
)
