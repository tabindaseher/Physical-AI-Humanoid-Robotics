@echo off
REM Script to index book content into the vector database

echo Indexing book content into vector database...

cd backend

REM Run the ingestion script
python ingest_books.py

echo Book content has been indexed into the vector database.
echo You can now start the backend server with: run_backend.bat