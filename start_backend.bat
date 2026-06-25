@echo off
REM ============================================================
REM  CathSim backend launcher  (double-click to start)
REM  Starts the FastAPI/uvicorn server on http://localhost:9000
REM ============================================================
title CathSim Backend (port 9000)
cd /d "%~dp0"

set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo [!] venv Python not found at "%PY%"
    echo     Falling back to system "python".
    set "PY=python"
)

echo Starting CathSim backend...
echo   URL:   http://localhost:9000
echo   Docs:  http://localhost:9000/docs
echo   Press Ctrl+C to stop.
echo.

"%PY%" -m services.main

echo.
echo [Backend stopped] exit code %errorlevel%
pause
