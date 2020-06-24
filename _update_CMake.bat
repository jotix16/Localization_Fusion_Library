::
:: IAV GmbH 2018
::
@ECHO OFF
cls
setlocal enabledelayedexpansion
set OWN_PATH=%~dp0
set BUILD_DIR=%OWN_PATH%build
set CMAKE_SRC=%OWN_PATH%
set COMPILER="Visual Studio 10 Win64"

:: CMake version:
set CMAKE_VER=cmake-3.13.3-win64-x64
:: When using a different cmake version , make sure that following link actually provides it!
set CMAKE_URL="http://fra-app-033.iavgroup.local:8000/s/s8LmLP2DKDZNsHJ/download"

set CMAKE_OUTDIR=%LIBRARIES_ROOT%\%CMAKE_VER%
set CMAKE_BIN=%CMAKE_OUTDIR%\bin\cmake.exe
set SEVEN_ZIP_EXE="C:\\Program Files\\7-Zip\\7z.exe"
set CMAKE_ZIPFILE="%LIBRARIES_ROOT%\%CMAKE_VER%.7z"


:: Replace / with \ in variable name
SET T2P=\
SET CMAKE_ZIPFILE=%CMAKE_ZIPFILE:/=!T2P!%

:: Do some checks:
IF NOT EXIST "%LIBRARIES_ROOT%" (
	goto no_lib_root
)

IF NOT EXIST %SEVEN_ZIP_EXE% (
	goto no_7zip_exe
)

IF EXIST %CMAKE_BIN% (
	goto have_cmake
)

if exist %CMAKE_OUTDIR% (
	goto unzip_cmake
)

:download_cmake
if exist %CMAKE_ZIPFILE% (
	echo File exists: '%CMAKE_ZIPFILE%'. Not downloading cmake
) else (
	echo Downloading CMake %CMAKE_ZIPFILE% from NextCloud
	if exist "C:\Program Files\Git\mingw64\bin\curl.exe" (
		echo Using curl
		"C:\Program Files\Git\mingw64\bin\curl.exe" %CMAKE_URL% > %CMAKE_ZIPFILE%
	) else (
		echo Using bitsadmin
		bitsadmin.exe /rawreturn /transfer "JobName" %CMAKE_URL% %CMAKE_ZIPFILE%
	)
)

:unzip_cmake
if exist %CMAKE_OUTDIR% (
	echo CMake dir exists: '%CMAKE_OUTDIR%'. Will not unpack it again.
) else (
	echo Unpacking CMake
	%SEVEN_ZIP_EXE% x -y "%CMAKE_ZIPFILE%" "-o%LIBRARIES_ROOT%
)

:have_cmake

:: create build dir
IF NOT EXIST "%BUILD_DIR%" (
    mkdir "%BUILD_DIR%"
    IF NOT EXIST "%BUILD_DIR%" (
        ECHO ^^! Could not create build directory
        goto error1
    )
)

:: enter build directory
cd "%BUILD_DIR%"

echo -- Start update of cmake
echo.

:: start cmake
"%CMAKE_BIN%" -G %COMPILER% "%CMAKE_SRC%"
:: check for errors during cmake
IF errorlevel 1 goto error1
goto noerror

:error1
echo.
echo == ERROR OCCURED DURING UPDATE OF CMAKE
pause
goto END

:no_lib_root
color fc
echo == ERROR: Missing environment variable LIBRARIES_ROOT.
echo == Please set this variable to an *existing* directory. Example: "D:/SDK"
echo == The AD-Platform will place all external libraries there.
echo .
pause
goto END

:no_lib_root
color fc
echo == ERROR: Missing 7-Zip program.
echo == Please install 7-Zip from SoftwareCenter.
echo .
pause
goto END

:noerror
echo.
echo == Update finished

:END
color 0f
cd "%OWN_PATH%"

pause
