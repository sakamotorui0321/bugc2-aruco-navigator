@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo [BugC2] Creating Python virtual environment...
  py -3 -m venv .venv || exit /b 1
)

".venv\Scripts\python.exe" -c "import cv2, numpy, psutil; assert hasattr(cv2, 'aruco')" >nul 2>&1
if errorlevel 1 (
  echo [BugC2] Installing required libraries. This can take several minutes...
  ".venv\Scripts\python.exe" -m pip install --upgrade pip || exit /b 1
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt || exit /b 1
)

echo [BugC2] Starting in DRY-RUN mode. Press E in the video window to arm UDP.
".venv\Scripts\python.exe" main.py %*
exit /b %errorlevel%
