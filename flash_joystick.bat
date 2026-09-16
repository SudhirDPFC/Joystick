@echo off
echo ====================================================================
echo   Ci4UVS -- Flash Native USB HID Gamepad (STM32CubeIDE Build)
echo ====================================================================

set STM32_PROG="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set BIN_FILE=%~dp0Debug\JOYSTICK.bin

if not exist %BIN_FILE% (
    echo [ERROR] %BIN_FILE% not found! Please build the project in STM32CubeIDE first.
    pause
    exit /b 1
)

echo [1/1] Flashing %BIN_FILE% via ST-Link...
%STM32_PROG% -c port=SWD mode=HOTPLUG freq=4000 -w "%BIN_FILE%" 0x08000000 -v -rst
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [RETRY] Attempting normal Under-Reset flash...
    %STM32_PROG% -c port=SWD mode=UR -w "%BIN_FILE%" 0x08000000 -v -rst
)

echo.
echo ====================================================================
echo DONE! Open Windows Game Controllers (joy.cpl) to test the Gamepad!
echo ====================================================================
pause
