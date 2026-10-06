@echo off
rem ============================================================
rem FASTLANE - upload changed files to the ESP32 (host PC only)
rem
rem   upload_to_esp32.bat [--port COM5] [--no-reset] [--boot-log-sec N] FILE [FILE ...]
rem   example:
rem   upload_to_esp32.bat main.py config.py components\vl53l0x.py
rem
rem Paths are relative to this folder. Result + boot log -> upload_log.txt
rem Thonny must be DISCONNECTED first (Run > Disconnect, or close Thonny).
rem ============================================================
setlocal
cd /d "%~dp0"

set "PYEXE="
where py >nul 2>&1 && set "PYEXE=py -3"
if not defined PYEXE where python >nul 2>&1 && set "PYEXE=python"
if not defined PYEXE if exist "%LOCALAPPDATA%\Programs\Thonny\python.exe" set "PYEXE="%LOCALAPPDATA%\Programs\Thonny\python.exe""
if not defined PYEXE if exist "%ProgramFiles%\Thonny\python.exe" set "PYEXE="%ProgramFiles%\Thonny\python.exe""
if not defined PYEXE if exist "%ProgramFiles(x86)%\Thonny\python.exe" set "PYEXE="%ProgramFiles(x86)%\Thonny\python.exe""

if not defined PYEXE (
  echo RESULT: FAILED - no Python found ^(install Python or Thonny^) > "%~dp0upload_log.txt"
  type "%~dp0upload_log.txt"
  exit /b 9
)

%PYEXE% "%~dp0_host_tools\esp32_upload.py" %*
set "RC=%ERRORLEVEL%"
endlocal & exit /b %RC%
