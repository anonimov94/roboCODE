@echo off
rem Test roboCODE.exe bez interakcji. Log: out_test\test_exe.log
chcp 65001 >nul
cd /d "%~dp0"
if not exist roboCODE.exe ( echo Brak roboCODE.exe - najpierw build_exe.bat & pause & exit /b 1 )
if exist out_test rmdir /s /q out_test
mkdir out_test
set LOG=out_test\test_exe.log
echo === TEST roboCODE.exe %DATE% %TIME% === > %LOG%
echo --- 1 demo kuka >> %LOG%
roboCODE.exe demo --out out_test\demo_kuka >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% >> %LOG%
echo --- 2 demo abb >> %LOG%
roboCODE.exe demo --post abb --out out_test\demo_abb >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% >> %LOG%
echo --- 3 samples >> %LOG%
roboCODE.exe samples out_test\samples >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% >> %LOG%
for %%f in (out_test\samples\*.nc) do call :machine "%%f" "%%~nf"
echo --- 5 blokada: nieobslugiwany kod G41 >> %LOG%
(echo G21& echo G41 D1& echo G1 X10 F100) > out_test\bad.nc
roboCODE.exe machine out_test\bad.nc --out out_test\bad >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% (oczekiwane 2) >> %LOG%
echo --- 6 raport >> %LOG%
roboCODE.exe report out_test\demo_kuka\Machining.manifest.json >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% >> %LOG%
echo --- 7 menu: wyjscie >> %LOG%
echo 0| roboCODE.exe >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% >> %LOG%
echo === KONIEC === >> %LOG%
type %LOG%
exit /b 0

:machine
echo --- 4 machine %~2 >> %LOG%
roboCODE.exe machine %1 --out "out_test\nc_%~2" >> %LOG% 2>&1
echo EXIT=%ERRORLEVEL% >> %LOG%
exit /b
