# Quickstart Guide: Cohere-based RAG Chatbot Backend

## Prerequisites

- Python 3.11+
- Access to Cohere API (COHERE_API_KEY)
- Qdrant Cloud account and cluster (QDRANT_URL, QDRANT_API_KEY, QDRANT_CLUSTER_ID)
- Neon Serverless Postgres database (NEON_DATABASE_URL)

## Setup Environment

1. Create a virtual environment:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables in `.env` file:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_CLUSTER_ID=your_qdrant_cluster_id
   NEON_DATABASE_URL=your_neon_database_url
   ```

## Initialize the Application

1. Start the FastAPI server:
   ```bash
   cd backend
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

2. The API will be available at `http://localhost:8000`

## API Endpoints

### Health Check
- `GET /health` - Check if the service is running

### RAG Endpoints
- `POST /rag/book-wide` - Ask questions about the entire book
- `POST /rag/selected-text` - Ask questions about user-provided text only

### Example Usage

**Book-wide Q&A:**
```bash
curl -X POST http://localhost:8000/rag/book-wide \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key concepts in chapter 3?",
    "session_id": "unique-session-id"
  }'
```

**Selected-text Q&A:**
```bash
curl -X POST http://localhost:8000/rag/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept?",
    "selected_text": "The concept of embodied AI involves...",
    "session_id": "unique-session-id"
  }'
```

## Development Workflow

1. **Index Book Content**: Use the chunking utilities to process book content into searchable chunks
2. **Test Retrieval**: Verify that similarity search returns relevant results
3. **Validate Responses**: Ensure answers are grounded in the provided context
4. **Test Context Isolation**: Confirm selected-text mode ignores global context

## Testing

Run unit tests:
```bash
pytest tests/
```

Run integration tests:
```bash
pytest tests/integration/
```

## Architecture Overview

The system follows an agent-first design with:
- Retrieval Agent: Handles content search and similarity matching
- Context Validator: Ensures responses stay within provided context
- Answer Generator: Creates responses using Cohere based on retrieved context

## Troubleshooting

- **API Key Issues**: Verify all environment variables are set correctly
- **Vector Search Problems**: Check Qdrant cluster connectivity and collection configuration
- **Database Connection**: Ensure Neon Postgres URL is properly formatted