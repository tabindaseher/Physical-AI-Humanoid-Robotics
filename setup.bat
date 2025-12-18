@echo off
REM Setup script for the Physical AI & Humanoid Robotics RAG Chatbot

echo Setting up the Physical AI & Humanoid Robotics RAG Chatbot...

REM Install Python dependencies
echo Installing Python dependencies...
cd backend

REM Create virtual environment if it doesn't exist
if not exist ".venv" (
    echo Creating virtual environment...
    python -m venv .venv
)

REM Activate virtual environment and install dependencies
if exist ".venv\Scripts\activate.bat" (
    call .venv\Scripts\activate.bat
) else (
    echo Warning: Virtual environment not found, installing globally
)

pip install -r requirements.txt

REM Check if Qdrant is available via Docker
echo Checking for Docker installation...
docker --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Docker is not installed or not in PATH.
    echo Please install Docker Desktop from: https://www.docker.com/products/docker-desktop
    echo.
    echo To continue setup without Docker, install Qdrant manually:
    echo 1. Visit: https://qdrant.tech/documentation/quick-start/
    echo 2. Run Qdrant locally on port 6333
    echo.
    echo Press any key to continue with manual Qdrant setup...
    pause >nul
) else (
    REM Start Qdrant container
    echo Starting Qdrant vector database...
    docker stop qdrant >nul 2>&1
    docker run -d --name qdrant -p 6333:6333 qdrant/qdrant
    echo Waiting for Qdrant to start...
    timeout /t 15 /nobreak >nul

    REM Check if Qdrant is running
    curl -s http://localhost:6333/dashboard >nul 2>&1
    if %errorlevel% equ 0 (
        echo Qdrant is running successfully on port 6333
    ) else (
        echo Warning: Qdrant may not be running properly
        echo Trying to check again...
        timeout /t 5 /nobreak >nul
        curl -s http://localhost:6333/dashboard >nul 2>&1
        if %errorlevel% equ 0 (
            echo Qdrant is running successfully on port 6333
        ) else (
            echo Error: Qdrant is not accessible on port 6333
        )
    )
)

REM Create data directory if it doesn't exist
if not exist "data" mkdir data

REM Run the ingestion script to load book content into vector database
echo.
echo Loading book content into vector database...
python ingest_books.py

REM Verify the backend can start properly
echo.
echo Testing backend server startup...
python -c "import main; print('Backend modules loaded successfully')" 2>nul
if %errorlevel% equ 0 (
    echo Backend modules loaded successfully
) else (
    echo Error: Backend modules failed to load
    echo Please check the backend installation and dependencies
)

echo.
echo Setup complete! Here's how to run the RAG Chatbot:
echo.
echo 1. Start the backend server:
echo    cd backend
echo    uvicorn main:app --reload --host 0.0.0.0 --port 8000
echo.
echo 2. In a new terminal, start the frontend:
echo    cd ..
echo    npm install
echo    npm start
echo.
echo 3. Access the chatbot at http://localhost:3000
echo.
echo Make sure Qdrant is running on port 6333 for full functionality.
echo Press any key to continue...
pause >nul