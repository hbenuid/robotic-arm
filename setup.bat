@echo off
setlocal EnableExtensions
cd /d "%~dp0"

echo === Robotic-arm Windows setup ===
echo.

REM Install uv if not already on PATH
where uv >nul 2>nul
if %errorlevel% NEQ 0 (
    echo uv not found. Installing...
    powershell -ExecutionPolicy ByPass -NoProfile -Command "irm https://astral.sh/uv/install.ps1 | iex"
    if errorlevel 1 (
        echo.
        echo ERROR: uv install failed.
        pause
        exit /b 1
    )
    REM uv installs to %USERPROFILE%\.local\bin; add to current session PATH
    set "PATH=%USERPROFILE%\.local\bin;%PATH%"
) else (
    for /f "tokens=*" %%v in ('uv --version') do echo Found %%v
)

echo.
echo Running uv sync...
uv sync
if errorlevel 1 (
    echo.
    echo ERROR: uv sync failed.
    pause
    exit /b 1
)

echo.
echo === Setup complete ===
echo Open a NEW terminal and run:  uv run launcher
echo.
pause
