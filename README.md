# Cohere-based RAG Chatbot Backend

This backend implements a Retrieval-Augmented Generation (RAG) chatbot that answers questions strictly from book content with support for selected-text Q&A mode. The system uses FastAPI with Cohere API for embeddings and responses, Qdrant Cloud for vector storage, and Neon Serverless Postgres for session management.

## Architecture

The system follows an agent-first design with:
- Retrieval Agent: Handles content search and similarity matching
- Context Validator: Ensures responses stay within provided context
- Answer Generator: Creates responses using Cohere based on retrieved context

## Tech Stack

- **Framework**: FastAPI
- **LLM/Embeddings**: Cohere API
- **Vector Store**: Qdrant Cloud
- **Database**: Neon Serverless Postgres
- **Language**: Python 3.11

## Setup

1. Clone the repository
2. Navigate to the backend directory
3. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
4. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
5. Copy the environment template and configure your credentials:
   ```bash
   cp .env .env.local
   # Edit .env.local with your actual credentials
   ```
6. Start the development server:
   ```bash
   uvicorn main:app --reload
   ```

## Environment Variables

Create a `.env` file with the following variables:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_CLUSTER_ID=your_qdrant_cluster_id

# Neon Postgres Configuration
NEON_DATABASE_URL=your_neon_database_url

# Application Configuration
ENVIRONMENT=development
LOG_LEVEL=info
```

## API Endpoints

- `GET /health` - Health check endpoint
- `POST /rag/book-wide` - Book-wide Q&A endpoint
- `POST /rag/selected-text` - Selected-text Q&A endpoint

## Development

Run tests:
```bash
pytest
```

## Security & Error Handling

The application includes:
- Rate limiting (50 requests per hour per IP)
- CORS configuration
- Security headers
- Comprehensive error handling and logging
- Input validation and sanitization

## Architecture

The system follows an agent-first design with:
- Retrieval Agent: Handles content search and similarity matching
- Context Validator: Ensures responses stay within provided context
- Answer Generator: Creates responses using Cohere based on retrieved context
- Content Indexing Service: Manages book content chunking and storage
- Citation Service: Formats source references according to academic standards

## Features

1. **Book-wide Q&A**: Ask questions about the entire book content with source citations
2. **Selected-text Q&A**: Ask questions about user-provided text only (ignores global context)
3. **Context Validation**: Ensures all responses are grounded in provided context without hallucination"# Physical-AI-Backend" 
