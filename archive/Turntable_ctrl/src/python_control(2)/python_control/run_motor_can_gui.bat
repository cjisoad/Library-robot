@echo off
setlocal

cd /d "%~dp0"

set "PYTHON_CMD="

where py >nul 2>nul
if %errorlevel% equ 0 (
    py -0 >nul 2>nul
    if %errorlevel% equ 0 (
        set "PYTHON_CMD=py -3"
    )
)

if not defined PYTHON_CMD (
    where python >nul 2>nul
    if %errorlevel% equ 0 (
        set "PYTHON_CMD=python"
    )
)

if not defined PYTHON_CMD (
    echo Python was not found. Install Python 3 or point VS Code to a valid interpreter.
    pause
    exit /b 1
)

%PYTHON_CMD% -c "import can, serial" >nul 2>nul
if %errorlevel% neq 0 (
    echo Installing python-can and pyserial...
    %PYTHON_CMD% -m pip install python-can pyserial
    if %errorlevel% neq 0 (
        echo Dependency installation failed. Check your network or Python environment.
        pause
        exit /b 1
    )
)

%PYTHON_CMD% motor_can_gui.py
if %errorlevel% neq 0 (
    echo GUI failed to start. Check the error output above.
    pause
    exit /b 1
)

endlocal
