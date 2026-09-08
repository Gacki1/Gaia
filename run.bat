@echo off
rem Launch the built planet. Pass extra args through, e.g.:  run.bat --wireframe
set "EXE=%~dp0build\Release\planet.exe"
if not exist "%EXE%" (
    echo planet.exe not found. Build it first:  build.bat
    exit /b 1
)
"%EXE%" %*
