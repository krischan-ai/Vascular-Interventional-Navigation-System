@echo off
setlocal
cd /d "%~dp0"

if not defined CATHSIM_SERVER_URL set "CATHSIM_SERVER_URL=ws://192.168.1.107:9000/ws/session"

set "GODOT_EXE="
for /f "delims=" %%F in ('where godot 2^>nul') do if not defined GODOT_EXE set "GODOT_EXE=%%F"
for /f "delims=" %%F in ('where godot4 2^>nul') do if not defined GODOT_EXE set "GODOT_EXE=%%F"

if not defined GODOT_EXE (
    for %%F in ("%USERPROFILE%\Desktop\Godot*_win64.exe") do if exist "%%~fF" set "GODOT_EXE=%%~fF"
)
if not defined GODOT_EXE (
    for %%F in ("%USERPROFILE%\Downloads\Godot*_win64.exe") do if exist "%%~fF" set "GODOT_EXE=%%~fF"
)

if not defined GODOT_EXE (
    echo [!] Godot was not found in PATH, Desktop, or Downloads.
    echo     Install Godot 4.7 or run scripts\validate_godot.ps1 -GodotExe ^<path^>.
pause
    exit /b 1
)

echo Starting CathSim Godot client...
echo   Godot:  %GODOT_EXE%
echo   Project: %~dp0godot_client
echo   Backend: %CATHSIM_SERVER_URL%
start "CathSim Godot Client" "%GODOT_EXE%" --path "%~dp0godot_client" -- --server-url=%CATHSIM_SERVER_URL%
