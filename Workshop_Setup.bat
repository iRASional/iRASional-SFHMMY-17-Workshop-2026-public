@echo off
setlocal

echo ===================================================
echo    Workshop Setup: Installing Required Tools
echo ===================================================
echo.

:: 1. Check if Python is installed
python --version >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    echo [!] Python is not installed on this computer.
    echo [*] Downloading Python 3.11...
    curl -# -o python-installer.exe https://www.python.org/ftp/python/3.11.8/python-3.11.8-amd64.exe
    
    echo [*] Installing Python... Please wait, this may take a few minutes.
    start /wait python-installer.exe /quiet InstallAllUsers=0 PrependPath=1 Include_test=0
    del python-installer.exe
    
    echo [*] Python installed successfully!
    set PY_CMD=py
) ELSE (
    echo [*] Python is already installed!
    set PY_CMD=python
)

echo.
echo [*] Updating pip...
%PY_CMD% -m ensurepip --upgrade >nul 2>&1
%PY_CMD% -m pip install --upgrade pip >nul 2>&1

echo [*] Downloading and installing required libraries...
echo [*] This might take a few minutes depending on internet speed.
%PY_CMD% -m pip install ultralytics opencv-python matplotlib networkx pillow lxml

echo.
echo ===================================================
echo    Setup Complete! You are ready for the workshop.
echo ===================================================
echo You can now open the Python scripts to fill in the blanks.
echo Once finished, run 'run.py' to start the system.
echo.
pause