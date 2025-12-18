# Setting Up the RAG Chatbot for Physical AI & Humanoid Robotics Book

This guide will help you set up the RAG (Retrieval-Augmented Generation) chatbot that can answer questions about the book content.

## Prerequisites

- Python 3.8 or higher
- Node.js 18 or higher
- Docker (for Qdrant vector database) - **Required for full functionality**
- Git

## Step-by-Step Setup

### 1. Clone and Navigate to the Project

```bash
git clone <your-repo-url>
cd book  # or whatever your project directory is named
```

### 2. Install Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 3. Start the Qdrant Vector Database

```bash
# Option 1: Using Docker (recommended)
docker run -d --name qdrant -p 6333:6333 qdrant/qdrant

# Option 2: Manual installation (visit https://qdrant.tech/documentation/quick-start/)
# Note: On Windows without Docker, you may need to install Qdrant natively
```

### 4. Ingest the Book Content into the Vector Database

```bash
# Make sure you're in the backend directory
cd backend

# Run the ingestion script to load book content
python ingest_books.py
```

### 5. Start the Backend API Server

```bash
# Make sure you're in the backend directory
cd backend

# Start the API server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

You should see output indicating that services are initializing:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     RAG services initialized successfully with book content loaded
```

> **Note:** If Qdrant is not running, the server will start in "degraded" mode with limited functionality.

### 6. Start the Frontend (Docusaurus)

In a new terminal, from the main project directory:

```bash
cd book  # or your project root
npm install
npm start
```

### 7. Access the Chatbot

1. Open your browser to `http://localhost:3000`
2. Look for the chatbot icon 💬 in the bottom-right corner of the page
3. Click it to open the chat interface
4. Ask questions about the book!

## Windows-Specific Setup

Since you're on Windows (as noted in your project), there are some specific considerations:

### Installing Docker on Windows
1. Download and install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
2. Restart your computer after installation
3. Start Docker Desktop before proceeding with the setup

### Alternative: Running Without Docker
If Docker is not available, you have a few options:
1. **Limited functionality**: The backend will still run but without RAG capabilities
2. **Local Qdrant**: Install Qdrant natively on Windows (see official documentation)
3. **Use Colab**: Run the backend on Google Colab with GPU support

## Troubleshooting

### Common Issues and Solutions

#### Issue: "Backend service is not available"
**Solution:** Verify the backend server is running:
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

#### Issue: Qdrant not connecting (Windows)
**Solution:** Make sure Docker Desktop is running and Qdrant is started:
```cmd
docker run -d --name qdrant -p 6333:6333 qdrant/qdrant
```

Check if Qdrant is running:
```cmd
docker ps
curl http://localhost:6333/dashboard
```

#### Issue: "RAG service is not currently available" or "degraded" status
**Solution:** The vector database isn't connected. Check:
1. Is Docker Desktop running?
2. Is Qdrant container running? (`docker ps`)
3. Has the book content been ingested? (`python ingest_books.py`)

Verify with health check:
```bash
curl http://localhost:8000/api/v1/health
```

#### Issue: PyTorch DLL errors on Windows
**Solution:** Reinstall PyTorch with CPU-only version:
```bash
pip uninstall torch torchvision torchaudio
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

#### Issue: Book content not found
**Solution:** Make sure the docs path is correct and re-run ingestion:
```bash
cd backend
python ingest_books.py
```

### Verification Steps

1. Check backend health:
   ```bash
   curl http://localhost:8000/api/v1/health
   ```

2. Verify Qdrant connection (when running):
   ```bash
   curl http://localhost:6333/dashboard
   ```

3. Test the API:
   ```bash
   curl -X POST http://localhost:8000/api/v1/rag/book-wide \
     -H "Content-Type: application/json" \
     -d '{"query": "What is Physical AI?", "session_id": "test"}'
   ```

## Automatic Setup Script

Run the automated setup script from the main project directory:

```cmd
# On Windows:
setup.bat
```

This will:
1. Create a virtual environment
2. Install dependencies
3. Start Qdrant with Docker
4. Ingest book content
5. Verify the system

## Using the Chatbot

### In the Frontend
- The chatbot appears as a 💬 icon in the bottom-right corner
- Click to open the chat interface
- The button color indicates status:
  - Blue: Fully functional
  - Gray: Checking connection
  - Red: Backend unavailable
- The status badge shows server health (Online/Offline)

### API Endpoints
- Health check: `GET http://localhost:8000/api/v1/health`
- Book-wide Q&A: `POST http://localhost:8000/api/v1/rag/book-wide`

Example API call:
```bash
curl -X POST http://localhost:8000/api/v1/rag/book-wide \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key concepts in Physical AI?",
    "session_id": "my-session-123"
  }'
```

## Configuration Options

### Environment Variables
Create a `.env` file in the backend directory to set:
- `OPENAI_API_KEY`: For OpenAI-based answer generation (optional, fallback uses rule-based generation)

### Model Configuration
The system uses:
- Embedding model: `all-MiniLM-L6-v2` (for sentence similarity)
- Answer generation: OpenAI GPT-3.5-turbo (with fallback)

## Usage Tips

- **With Qdrant running**: Full RAG functionality with context retrieval from the book
- **Without Qdrant**: Rule-based answers using book content (limited effectiveness)
- Sources are displayed to show which parts of the book the answer was derived from
- If you update the book content, re-run the ingestion script to update the vector database
- The system automatically handles context from across the entire book

## Stopping Services

To stop the services:
1. Press `Ctrl+C` in the backend terminal to stop the API server
2. Press `Ctrl+C` in the frontend terminal to stop Docusaurus
3. To stop Qdrant: `docker stop qdrant`

To stop all running processes:
```cmd
# Stop backend servers
taskkill /f /im uvicorn.exe
taskkill /f /im python.exe /fi "IMAGENAME eq python.exe" /fi "WINDOWTITLE eq *main*"

# Stop Qdrant container
docker stop qdrant
```

## Performance Notes

- Initial startup may take 30-60 seconds as embeddings are loaded
- First queries may be slower as the system initializes
- Book content has been chunked and indexed for efficient retrieval
- The system can handle multiple concurrent users

---

For any issues not covered here, please check the console logs in your browser's developer tools and the backend server logs. The chatbot interface includes status indicators and helpful error messages to assist with troubleshooting.