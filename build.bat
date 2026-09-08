@echo off
setlocal enabledelayedexpansion

rem ==========================================================================
rem  Procedural Planet -- one-command build.
rem  Auto-loads the Visual Studio x64 dev environment (cl + Windows SDK) and
rem  uses the Ninja that ships with Visual Studio, so you do NOT need to open a
rem  developer command prompt yourself.
rem
rem      build.bat                :: Release build  -> build\Release\planet.exe
rem      build.bat Debug          :: Debug build (Vulkan validation on)
rem      build.bat Release run    :: build, then launch
rem ==========================================================================

set "ROOT=%~dp0"
set "BUILD=%ROOT%build"
set "VCVARS=F:\Microsoft Visual Studio\VC\Auxiliary\Build\vcvars64.bat"
set "NINJA=F:\Microsoft Visual Studio\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

set "CONFIG=%~1"
if "%CONFIG%"=="" set "CONFIG=Release"

echo ============================================================
echo  Procedural Planet build (%CONFIG%)
echo ============================================================

rem --- Load the VS x64 dev environment if cl is not already available. ---
where cl >nul 2>nul
if errorlevel 1 (
    if not exist "%VCVARS%" (
        echo [Build] ERROR: vcvars64.bat not found at "%VCVARS%".
        echo         Open an "x64 Native Tools Command Prompt" and re-run, or fix the path.
        exit /b 1
    )
    echo [Build] Loading Visual Studio developer environment...
    call "%VCVARS%" >nul
    where cl >nul 2>nul
    if errorlevel 1 (
        echo [Build] ERROR: could not initialise the MSVC toolchain.
        exit /b 1
    )
)

rem --- Pick a Ninja (bundled with VS, else whatever is on PATH). ---
if not exist "%NINJA%" (
    for /f "delims=" %%i in ('where ninja 2^>nul') do set "NINJA=%%i"
)
if not exist "%NINJA%" (
    echo [Build] ERROR: ninja.exe not found. Install Ninja or fix the path.
    exit /b 1
)

echo.
echo [1/2] Configuring (Ninja Multi-Config)...
cmake -S "%ROOT%." -B "%BUILD%" -G "Ninja Multi-Config" -DCMAKE_MAKE_PROGRAM="%NINJA%"
if errorlevel 1 exit /b 1

echo.
echo [2/2] Building %CONFIG%...
cmake --build "%BUILD%" --config %CONFIG%
if errorlevel 1 exit /b 1

echo.
echo ============================================================
echo  Build OK:  %BUILD%\%CONFIG%\planet.exe
echo ============================================================

if /i "%~2"=="run" (
    echo [Build] Launching...
    pushd "%BUILD%\%CONFIG%"
    start "" "planet.exe"
    popd
)
exit /b 0
