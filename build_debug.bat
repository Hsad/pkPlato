@echo off

set OUT_DIR=build\debug

if not exist %OUT_DIR% mkdir %OUT_DIR%

odin build source\main_release -out:%OUT_DIR%\game_debug.exe -strict-style -vet -debug
IF %ERRORLEVEL% NEQ 0 exit /b 1

xcopy /y /e /i assets %OUT_DIR%\assets > nul
IF %ERRORLEVEL% NEQ 0 exit /b 1
