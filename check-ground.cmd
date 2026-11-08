@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Create .venv and install requirements.txt first.
 exit /b 1
)
".venv\Scripts\python.exe" scripts\check_ground.py --label local
exit /b %errorlevel%
