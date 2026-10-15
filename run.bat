@echo off
setlocal
cd /d "%~dp0"
REM Adjust MSYS2 installation directory if yours is different.
if exist "C:\MCA Projects\VisionGuidedLanding\ucrt64\bin\g++.exe" (
  set "PATH=C:\MCA Projects\VisionGuidedLanding\ucrt64\bin;%PATH%"
) else (
  set "PATH=C:\msys64\ucrt64\bin;%PATH%"
)
if not exist build mkdir build
cmake -S . -B build -G Ninja
if errorlevel 1 goto failed
cmake --build build
if errorlevel 1 goto failed
build\drone.exe demo 30
if errorlevel 1 goto failed
build\drone.exe marker
if errorlevel 1 goto failed
echo.
echo Done. Check results folder.
pause
exit /b 0
:failed
echo Build or run failed. See message above.
pause
exit /b 1
