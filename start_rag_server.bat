@echo off
REM Script to run the backend server for the RAG chatbot
echo Starting the Physical AI & Humanoid Robotics RAG Chatbot backend...

REM Check if we're already in the backend directory
if not exist "main.py" (
    echo Error: This script must be run from the backend directory.
    echo Navigate to the backend folder and run this script again.
    pause
    exit /b 1
)

REM Check if Python is available
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Python is not installed or not in PATH.
    echo Please install Python 3.8 or higher and add it to PATH.
    pause
    exit /b 1
)

REM Check if required dependencies are installed
echo Checking for required dependencies...
python -c "import fastapi" >nul 2>&1
if %errorlevel% neq 0 (
    echo FastAPI not found. Installing dependencies...
    pip install -r requirements.txt
)

REM Check if Qdrant is running
echo Checking for Qdrant vector database at localhost:6333...
curl -s http://localhost:6333/dashboard >nul 2>&1
if %errorlevel% neq 0 (
    echo WARNING: Qdrant is not running on localhost:6333
    echo To start Qdrant, run in another terminal:
    echo docker run -d --name qdrant -p 6333:6333 qdrant/qdrant
    echo.
    echo The backend will start but with limited functionality.
    echo Press any key to continue anyway...
    pause >nul
)

REM Test if main module loads correctly
echo Testing main module...
python -c "import main; print('Main module loaded successfully')" 2>nul
if %errorlevel% neq 0 (
    echo Error: Failed to load main module. Check for dependency issues.
    pause
    exit /b 1
)

echo.
echo Starting backend server on http://localhost:8000...
echo.
echo NOTE: If Qdrant is not running, the service will start but responses may be limited
echo.
echo Press Ctrl+C to stop the server
echo.

REM Start the backend server with auto-reload
uvicorn main:app --reload --host 0.0.0.0 --port 8000

echo Server stopped.
pause