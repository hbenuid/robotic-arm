@echo off
setlocal EnableExtensions
cd /d "%~dp0"

REM Ensure uv is on PATH for this session even if a recent setup hasn't
REM been picked up by File Explorer's environment yet.
set "PATH=%USERPROFILE%\.local\bin;%PATH%"

where uv >nul 2>nul
if %errorlevel% NEQ 0 (
    echo uv not found on PATH.
    echo Run setup.bat first to install uv and project dependencies.
    echo.
    pause
    exit /b 1
)

uv run launcher

echo.
echo Motor controller exited.
pause
