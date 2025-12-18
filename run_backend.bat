@echo off
REM Script to run the backend server for the RAG chatbot
echo Starting the Physical AI & Humanoid Robotics RAG Chatbot backend...

REM Navigate to backend directory
cd backend

REM Start the backend server with auto-reload
echo Starting backend server on http://localhost:8000...
echo NOTE: If Qdrant is not running, the service will start but with limited functionality
echo Make sure Qdrant is available at localhost:6333
python -c "import main; print('Main module loaded successfully')" 2>nul

echo.
echo Running uvicorn server...
uvicorn main:app --reload --host 0.0.0.0 --port 8000

echo Server stopped.
pause