@echo off
rem Uruchamia launcher roboCODE bez budowania exe (wymaga .venv – patrz build_exe.bat).
chcp 65001 >nul
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (".venv\Scripts\python.exe" roboCODE.py %*) else (py -3 roboCODE.py %*)
if errorlevel 1 pause
