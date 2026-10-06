@echo off
rem FASTLANE - record the ESP32 console (no reset) -> monitor_log.txt
rem   monitor_esp32.bat [--port COM7] [--sec 60]
rem Thonny must be DISCONNECTED first.
setlocal
cd /d "%~dp0"

set "PYEXE="
where py >nul 2>&1 && set "PYEXE=py -3"
if not defined PYEXE where python >nul 2>&1 && set "PYEXE=python"
if not defined PYEXE if exist "%LOCALAPPDATA%\Programs\Thonny\python.exe" set "PYEXE="%LOCALAPPDATA%\Programs\Thonny\python.exe""
if not defined PYEXE if exist "%ProgramFiles%\Thonny\python.exe" set "PYEXE="%ProgramFiles%\Thonny\python.exe""
if not defined PYEXE if exist "%ProgramFiles(x86)%\Thonny\python.exe" set "PYEXE="%ProgramFiles(x86)%\Thonny\python.exe""

if not defined PYEXE (
  echo RESULT: FAILED - no Python found ^(install Python or Thonny^) > "%~dp0monitor_log.txt"
  type "%~dp0monitor_log.txt"
  exit /b 9
)

%PYEXE% "%~dp0_host_tools\esp32_monitor.py" %*
set "RC=%ERRORLEVEL%"
if "%~1"=="" timeout /t 5 >nul
endlocal & exit /b %RC%
