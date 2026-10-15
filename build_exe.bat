@echo off
rem Buduje roboCODE.exe (menu) i roboCODE-GUI.exe (okno GUI) w folderze projektu. Uruchom dwuklikiem.
rem Na koniec tworzy skrot "roboCODE GUI" w folderze projektu i na pulpicie.
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

echo Instaluje zaleznosci (z GUI: PySide6, pyvista - za pierwszym razem kilka minut) ...
".venv\Scripts\python.exe" -m pip install --upgrade pip >nul
".venv\Scripts\python.exe" -m pip install -e .[gui] pyinstaller || goto :error

echo.
echo [1/2] Buduje roboCODE.exe (menu) ...
".venv\Scripts\python.exe" -m PyInstaller --noconfirm --clean --onefile --console ^
    --name roboCODE --collect-data robocode ^
    --exclude-module PySide6 --exclude-module pyvista --exclude-module pyvistaqt ^
    --exclude-module vtk --exclude-module vtkmodules ^
    --exclude-module matplotlib --exclude-module pytest ^
    roboCODE.py || goto :error

echo.
echo [2/2] Buduje roboCODE-GUI.exe (okno GUI) ...
".venv\Scripts\python.exe" -m PyInstaller --noconfirm --clean --onedir --windowed ^
    --name roboCODE-GUI --collect-data robocode --collect-data pyvista --collect-all pyvistaqt ^
    --exclude-module pytest --exclude-module tkinter ^
    --exclude-module PySide6.QtWebEngineCore --exclude-module PySide6.QtWebEngineWidgets ^
    roboCODE_GUI.py || goto :error

echo.
echo Sprawdzam, czy GUI startuje ...
set QT_QPA_PLATFORM=offscreen
"dist\roboCODE-GUI\roboCODE-GUI.exe" --smoke
if errorlevel 1 (
    set QT_QPA_PLATFORM=
    echo BLAD: roboCODE-GUI.exe nie wystartowal. Szczegoly: %CD%\dist\out\gui_error.log
    pause
    exit /b 1
)
set QT_QPA_PLATFORM=

copy /y "dist\roboCODE.exe" "roboCODE.exe" >nul
if errorlevel 1 (
    echo.
    echo BLAD: nie mozna podmienic roboCODE.exe - jest uruchomiony. Zamknij okno roboCODE i uruchom ponownie.
    pause
    exit /b 1
)
robocopy "dist\roboCODE-GUI" "roboCODE-GUI" /MIR /NFL /NDL /NJH /NJS /NP >nul
if errorlevel 8 (
    echo.
    echo BLAD: nie mozna podmienic folderu roboCODE-GUI - GUI jest otwarte. Zamknij okno GUI i uruchom ponownie.
    pause
    exit /b 1
)

echo Tworze skroty "roboCODE GUI" ...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$w=New-Object -ComObject WScript.Shell; foreach($d in @('%CD%',[Environment]::GetFolderPath('Desktop'))){ $s=$w.CreateShortcut((Join-Path $d 'roboCODE GUI.lnk')); $s.TargetPath='%CD%\roboCODE-GUI\roboCODE-GUI.exe'; $s.WorkingDirectory='%CD%'; $s.Save() }" >nul 2>&1

echo.
echo Gotowe:
echo   GUI:  dwuklik "roboCODE GUI" (pulpit / folder projektu) albo roboCODE-GUI\roboCODE-GUI.exe
echo   Menu: %CD%\roboCODE.exe  (opcja 5 tez otwiera GUI)
echo Wyniki programu beda zapisywane w folderze "out".
pause
exit /b 0

:error
echo.
echo BLAD budowania - przewin w gore, aby zobaczyc komunikat.
pause
exit /b 1
