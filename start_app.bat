@echo off
REM Script to start both backend and frontend for the RAG Chatbot

echo Starting Physical AI & Humanoid Robotics RAG Chatbot...

echo.
echo Checking prerequisites...

REM Check if backend directory exists
if not exist "backend" (
    echo Error: backend directory not found!
    echo Please run this script from the project root directory.
    pause
    exit /b 1
)

REM Check if Qdrant is running
curl -s http://localhost:6333/dashboard >nul 2>&1
if %errorlevel% neq 0 (
    echo Warning: Qdrant is not running on port 6333
    echo Starting Qdrant with Docker (make sure Docker is installed)...
    docker run -d --name qdrant-temp -p 6333:6333 qdrant/qdrant
    timeout /t 10 /nobreak >nul
    curl -s http://localhost:6333/dashboard >nul 2>&1
    if %errorlevel% equ 0 (
        echo Qdrant started successfully
    ) else (
        echo Error: Could not start Qdrant. Please start it manually.
        pause
        exit /b 1
    )
) else (
    echo Qdrant is running on port 6333
)

echo.
echo Starting backend server in a new window...

REM Start backend server in a new window
start "RAG Backend" cmd /k "cd backend && if exist .venv\Scripts\activate.bat call .venv\Scripts\activate.bat && uvicorn main:app --reload --host 0.0.0.0 --port 8000"

timeout /t 5 /nobreak >nul

echo.
echo Installing frontend dependencies and starting Docusaurus...

REM Start frontend in the current window
npm install
npm start

echo.
echo Frontend stopped.
echo To restart, run: npm start
echo To stop the backend, close the backend window or press Ctrl+C in that window.
echo.
pause