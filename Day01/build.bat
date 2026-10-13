@echo off
chcp 65001 >nul
setlocal
pushd "%~dp0"
g++ -std=c++17 -g -Wall -Wextra -finput-charset=UTF-8 -fexec-charset=UTF-8 day01_angle_stats.cpp -o angle_stats.exe
if errorlevel 1 goto failed
g++ -std=c++17 -g -Wall -Wextra -finput-charset=UTF-8 -fexec-charset=UTF-8 day01_hellorobot.cpp -o hello.exe
if errorlevel 1 goto failed
echo Build OK: angle_stats.exe, hello.exe
popd
exit /b 0
:failed
popd
exit /b 1
