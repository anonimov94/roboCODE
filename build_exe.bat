@echo off
rem Buduje roboCODE.exe (launcher z menu) w folderze projektu. Uruchom dwuklikiem.
chcp 65001 >nul
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo Tworze srodowisko .venv ...
    py -3.12 -m venv .venv 2>nul || python -m venv .venv
)
if not exist ".venv\Scripts\python.exe" (
    echo BLAD: nie znaleziono Pythona. Zainstaluj Python 3.10+ z python.org i uruchom ponownie.
    pause
    exit /b 1
)

echo Instaluje zaleznosci ...
".venv\Scripts\python.exe" -m pip install --upgrade pip >nul
".venv\Scripts\python.exe" -m pip install -e . pyinstaller || goto :error

echo Buduje roboCODE.exe ...
".venv\Scripts\python.exe" -m PyInstaller --noconfirm --clean --onefile --console ^
    --name roboCODE --collect-data robocode ^
    --exclude-module PySide6 --exclude-module pyvista --exclude-module pyvistaqt ^
    --exclude-module matplotlib --exclude-module pytest ^
    roboCODE.py || goto :error

copy /y "dist\roboCODE.exe" "roboCODE.exe" >nul
if errorlevel 1 (
    echo.
    echo BLAD: nie mozna podmienic roboCODE.exe - jest uruchomiony. Zamknij okno roboCODE i uruchom ponownie.
    echo Nowa wersja jest w: %CD%\dist\roboCODE.exe
    pause
    exit /b 1
)
echo.
echo Gotowe: %CD%\roboCODE.exe
echo Wyniki programu beda zapisywane w folderze "out" obok exe.
pause
exit /b 0

:error
echo.
echo BLAD budowania - przewin w gore, aby zobaczyc komunikat.
pause
exit /b 1
