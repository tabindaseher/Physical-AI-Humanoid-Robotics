@echo off
REM Script to run Qdrant vector database locally as a service on Windows

echo Installing and starting Qdrant locally...

REM Download Qdrant for Windows if not already installed
if not exist "qdrant_server.exe" (
    echo Downloading Qdrant...
    echo Note: If the download fails, you can manually download Qdrant from:
    echo https://qdrant.tech/documentation/guides/installation/#local-installation
    
    REM Try to download qdrant for Windows
    REM Using curl to download from releases page (you might need to update version)
    REM Alternative: you may need to download manually from: 
    REM https://github.com/qdrant/qdrant/releases
    
    echo Please visit: https://github.com/qdrant/qdrant/releases
    echo Download the Windows release (qdrant-{version}-windows-x86_64.zip)
    echo Extract qdrant.exe and place it in the backend folder
    echo Then run this script again.
    pause
    exit /b 1
)

REM Start Qdrant server
echo Starting Qdrant server on port 6333...
start /min "" qdrant_server.exe --host 0.0.0.0 --port 6333

REM Wait a bit for the server to start
timeout /t 10 /nobreak >nul

REM Check if Qdrant is running
curl -s http://localhost:6333/dashboard >nul 2>&1
if %errorlevel% equ 0 (
    echo Qdrant is running successfully on port 6333
    echo You can now start the RAG backend server.
    echo.
    echo To test: curl http://localhost:6333/dashboard
    echo To stop Qdrant: taskkill /f /im qdrant_server.exe
) else (
    echo Qdrant may not be running properly. Check the qdrant_server.exe file exists.
    echo Make sure the server started without errors.
)

pause