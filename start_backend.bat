@echo off
REM ============================================================
REM  CathSim backend launcher  (double-click to start)
REM  Starts the FastAPI/uvicorn server on http://localhost:9000
REM ============================================================
title CathSim Backend (port 9000)
cd /d "%~dp0"

set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" (
    set "PY=%USERPROFILE%\.conda\envs\cathsim-dev\python.exe"
)
if not exist "%PY%" (
    echo [!] Project venv and cathsim-dev Python were not found.
    echo     Falling back to system "python".
    set "PY=python"
)

REM Warp's default long-path cache can fail with WinError 5 on Windows.
REM Keep a writable, versioned kernel cache inside the ignored .tmp folder.
set "WARP_CACHE_PATH=%~dp0.tmp\warp"
if not exist "%WARP_CACHE_PATH%" mkdir "%WARP_CACHE_PATH%"

REM Stable local Newton force-drive defaults.  Existing caller-provided values
REM still win, so developers can benchmark other profiles explicitly.
if not defined CATHSIM_NEWTON_SUBSTEPS set "CATHSIM_NEWTON_SUBSTEPS=8"
if not defined CATHSIM_NEWTON_ITERS set "CATHSIM_NEWTON_ITERS=4"
if not defined CATHSIM_NEWTON_SETTLE_STEPS set "CATHSIM_NEWTON_SETTLE_STEPS=480"
if not defined CATHSIM_NEWTON_RETURN_SETTLE_STEPS set "CATHSIM_NEWTON_RETURN_SETTLE_STEPS=120"
if not defined CATHSIM_NEWTON_ROTATE_SPEED set "CATHSIM_NEWTON_ROTATE_SPEED=5.0"

echo Starting CathSim backend...
echo   URL:   http://localhost:9000
echo   Docs:  http://localhost:9000/docs
echo   Python: %PY%
echo   Warp cache: %WARP_CACHE_PATH%
echo   Newton: %CATHSIM_NEWTON_SUBSTEPS%x%CATHSIM_NEWTON_ITERS% iterations, settle %CATHSIM_NEWTON_SETTLE_STEPS%
echo   Press Ctrl+C to stop.
echo.

"%PY%" -m services.main

echo.
echo [Backend stopped] exit code %errorlevel%
pause
