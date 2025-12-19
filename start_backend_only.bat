@echo off
REM Start the RAG Chatbot Backend Only - No Qdrant Required
echo Starting RAG Chatbot Backend Server (without Qdrant)...

REM Change to backend directory
cd /d "C:\Users\hp\Desktop\book\backend"

REM Start the backend server in a new window
start "RAG Backend Server" cmd /k "cd /d C:\Users\hp\Desktop\book\backend && python -m uvicorn main:app --host 127.0.0.1 --port 8000 --reload"

echo.
echo Waiting for server to start...
timeout /t 8 /nobreak >nul

echo.
echo Testing server connection...

REM Test if server is responding
curl -s http://localhost:8000/api/v1/health >nul 2>&1
if %errorlevel% equ 0 (
    echo SUCCESS: Server is running on http://localhost:8000
    echo The RAG chatbot backend is ready!
    echo.
    echo You can test the health endpoint at: http://localhost:8000/api/v1/health
    echo.
    echo The chatbot is running in fallback mode with your book content loaded.
    echo.
    echo To stop the server, close the backend window or press Ctrl+C in that window.
) else (
    echo Server is not responding. Please check the backend window for errors.
)

echo.
echo Press any key to continue...
pause >nul