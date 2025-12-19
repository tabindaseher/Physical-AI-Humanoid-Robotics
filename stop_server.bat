@echo off
REM Script to stop the RAG backend server
taskkill /f /im python.exe /fi "WINDOWTITLE eq *uvicorn*"
if %errorlevel% == 0 (
    echo Backend server stopped successfully.
) else (
    echo No backend server processes found or failed to stop them.
)