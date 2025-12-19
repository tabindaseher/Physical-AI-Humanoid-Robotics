@echo off
REM Start the RAG Chatbot Backend Server

echo Starting RAG Chatbot Backend Server...
echo NOTE: This will start the server on http://localhost:8000
echo Press Ctrl+C to stop the server

REM Change to backend directory
cd /d "C:\Users\hp\Desktop\book\backend"

REM Run the server
python -m uvicorn main:app --host 127.0.0.1 --port 8000 --reload