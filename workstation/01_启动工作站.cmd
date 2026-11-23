@echo off
chcp 65001 >nul
start "" /D "%~dp0app" "%~dp0app\颈部姿态三维工作站.exe"
if errorlevel 1 pause
