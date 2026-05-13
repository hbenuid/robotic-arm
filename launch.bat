@echo off
setlocal EnableExtensions
cd /d "%~dp0"

REM Ensure uv is on PATH for this session.
set "PATH=%USERPROFILE%\.local\bin;%PATH%"

REM Install uv on first run.
where uv >nul 2>nul
if %errorlevel% NEQ 0 (
    echo === First-time setup: installing uv ===
    powershell -ExecutionPolicy ByPass -NoProfile -Command "irm https://astral.sh/uv/install.ps1 | iex"
    if errorlevel 1 (
        echo.
        echo ERROR: uv install failed.
        pause
        exit /b 1
    )
    REM uv install updates persistent user PATH; refresh current session.
    set "PATH=%USERPROFILE%\.local\bin;%PATH%"
)

REM Sync dependencies (no-op if already current).
uv sync
if errorlevel 1 (
    echo.
    echo ERROR: uv sync failed.
    pause
    exit /b 1
)

uv run launcher

echo.
echo Motor controller exited.
pause
