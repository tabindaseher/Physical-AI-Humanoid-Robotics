import sys
import os
# Add the current directory to Python path to enable imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

import logging
from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, Any, List, Optional
from datetime import datetime
import uuid
from threading import Thread
import asyncio

# Import our RAG services
from services.embedding_service import EmbeddingService
from services.retrieval_service import RetrievalService
from services.answer_generation_service import AnswerGenerationService
from services.rag_service import RAGService

# Import the new RAG API router
try:
    from api.rag import router as rag_router
except ImportError as e:
    print(f"Warning: Could not import rag router: {e}")
    rag_router = None

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Initialize services globally so they can be reused
embedding_service = None
retrieval_service = None
answer_service = None
rag_service = None

def initialize_services():
    """Initialize all RAG services."""
    global embedding_service, retrieval_service, answer_service, rag_service

    # Initialize embedding service first
    try:
        embedding_service = EmbeddingService(model_name="all-MiniLM-L6-v2")
    except Exception as e:
        logger.error(f"Error initializing embedding service: {e}")
        embedding_service = None

    # Initialize retrieval service (Qdrant) - this might fail if Qdrant is not running
    try:
        retrieval_service = RetrievalService(
            host="localhost",
            port=6333,
            collection_name="book_embeddings",
            vector_size=embedding_service.get_embedding_dimension() if embedding_service else 384
        )
    except Exception as e:
        logger.warning(f"Qdrant not available: {e}")
        logger.info("RAG service will be unavailable until Qdrant is running")
        retrieval_service = None

    # Initialize answer generation service
    try:
        answer_service = AnswerGenerationService()
    except Exception as e:
        logger.error(f"Error initializing answer service: {e}")
        answer_service = None

    # Initialize RAG service only if dependencies are available
    if embedding_service and answer_service:  # Only require embedding and answer services to be available
        try:
            rag_service = RAGService(
                embedding_service=embedding_service,
                retrieval_service=retrieval_service,  # This can be None if Qdrant is not available
                answer_service=answer_service,
                docs_path="../docs"  # Path relative to backend directory
            )

            # Load and index book content (works for both vector DB and fallback)
            try:
                # Force load documents during initialization
                success = rag_service.load_and_index_book_content(force_reindex=False)
                if success:
                    logger.info(f"RAG services initialized successfully with book content loaded ({rag_service.get_document_count()} documents)")
                else:
                    logger.warning("RAG services initialized but no book content was loaded")
            except Exception as e:
                logger.error(f"Error loading book content: {e}")
                logger.info("RAG service initialized but may have limited functionality")
        except Exception as e:
            logger.error(f"Error initializing RAG service: {e}")
            rag_service = None
            logger.error("RAG service set to None due to initialization error")
    else:
        rag_service = None
        logger.info("RAG service not initialized due to missing dependencies")

# Initialize services when module is loaded
initialize_services()

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot API",
    description="API for RAG chatbot that answers questions from the Physical AI & Humanoid Robotics book",
    version="1.0.0"
)


@app.get("/")
async def root():
    """
    Root endpoint to provide API information and avoid infinite loading.
    """
    return {
        "message": "Physical AI & Humanoid Robotics RAG Chatbot API",
        "version": "1.0.0",
        "status": "running",
        "documentation": "/docs",
        "health": "/api/v1/health",
        "endpoints": {
            "health": "/api/v1/health",
            "book_wide_qa": "/api/v1/rag/book-wide",
            "selected_text_qa": "/api/v1/rag/selected-text"
        }
    }

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# RAG endpoints are now directly implemented in main.py


# Models
class HealthCheckResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: Dict[str, bool]
    document_count: int


class BookWideQueryRequest(BaseModel):
    query: str
    session_id: str
    metadata: Optional[Dict[str, Any]] = None


class SelectedTextQueryRequest(BaseModel):
    query: str
    selected_text: str
    session_id: str
    metadata: Optional[Dict[str, Any]] = None


class SourceReference(BaseModel):
    chunk_id: Optional[str] = None
    text_preview: str
    relevance_score: Optional[float] = None
    location: Optional[Dict[str, Any]] = None


class BookWideQueryResponse(BaseModel):
    answer: str
    sources: List[SourceReference]
    session_id: str
    confidence: Optional[float] = None
    retrieved_docs_count: int = 0


class SelectedTextQueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]  # For selected text mode
    session_id: str
    mode: str = "selected_text"
    confidence: Optional[float] = None


@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    global rag_service
    logger.info("Starting up RAG chatbot API")

    # At startup, try to ensure documents are loaded if rag_service exists
    if rag_service is not None:
        try:
            # Try to load content if not already loaded
            if rag_service.get_document_count() == 0:
                logger.info("Loading book content at startup...")
                success = rag_service.load_and_index_book_content(force_reindex=False)
                if success and rag_service.get_document_count() > 0:
                    logger.info(f"Successfully loaded {rag_service.get_document_count()} documents at startup")
                else:
                    logger.warning("No documents loaded at startup")
        except Exception as e:
            logger.error(f"Error loading book content at startup: {e}")
    else:
        # Try to initialize RAG service if not already done and dependencies exist
        if embedding_service is not None and answer_service is not None:
            try:
                logger.info("Initializing RAG service at startup...")
                rag_service = RAGService(
                    embedding_service=embedding_service,
                    retrieval_service=retrieval_service,  # This can be None if Qdrant is not available
                    answer_service=answer_service,
                    docs_path="../docs"  # Path relative to backend directory
                )

                # Load and index book content (works for both vector DB and fallback)
                try:
                    success = rag_service.load_and_index_book_content(force_reindex=False)
                    if success:
                        logger.info(f"RAG services initialized at startup with book content loaded ({rag_service.get_document_count()} documents)")
                    else:
                        logger.warning("RAG services initialized at startup but no book content was loaded")
                except Exception as e:
                    logger.error(f"Error loading book content at startup: {e}")
            except Exception as e:
                logger.error(f"Error initializing RAG service at startup: {e}")


@app.get("/api/v1/health", response_model=HealthCheckResponse)
async def health_check() -> HealthCheckResponse:
    """
    Health check endpoint to verify service and dependencies are accessible.
    """
    # Test Qdrant connectivity explicitly
    qdrant_healthy = False
    document_count = 0

    if retrieval_service is not None:
        try:
            document_count = retrieval_service.count_documents()
            qdrant_healthy = True
        except:
            qdrant_healthy = False
    else:
        # When using fallback mode, check RAG service document count
        if rag_service is not None:
            try:
                document_count = rag_service.get_document_count()
            except:
                document_count = 0

    # Check dependencies
    dependencies_status = {
        "qdrant": qdrant_healthy,
        "embedding_model": embedding_service is not None,
        "answer_generation": answer_service is not None,
        "rag_service": rag_service is not None
    }

    # Overall status - mark as healthy if core services are available
    # If in fallback mode and documents loaded, consider it healthy
    core_services_healthy = all([
        embedding_service is not None,
        answer_service is not None
    ])

    has_documents = document_count > 0
    rag_available = rag_service is not None

    overall_status = "healthy" if (core_services_healthy and (qdrant_healthy or (rag_available and has_documents))) else "degraded" if core_services_healthy else "unhealthy"

    return HealthCheckResponse(
        status=overall_status,
        timestamp=datetime.utcnow().isoformat(),
        dependencies=dependencies_status,
        document_count=document_count
    )


@app.post("/api/v1/rag/book-wide", response_model=BookWideQueryResponse)
async def book_wide_qa(request: BookWideQueryRequest):
    """
    Book-wide Q&A endpoint - Ask questions about the entire book content.
    """
    # Handle simple greetings and small talk without RAG
    query_lower = request.query.lower().strip()
    if query_lower in ["hi", "hello", "hey", "greetings", "good morning", "good afternoon", "good evening"]:
        response = BookWideQueryResponse(
            answer="Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the book content!",
            sources=[],
            session_id=request.session_id,
            confidence=1.0,
            retrieved_docs_count=0
        )
        return response

    if not rag_service:
        # When RAG service is not available, return error message
        logger.error("RAG service is not available")
        response = BookWideQueryResponse(
            answer="ERROR: No book content retrieved. RAG pipeline broken.",
            sources=[],
            session_id=request.session_id,
            confidence=0.0,
            retrieved_docs_count=0
        )
        return response

    try:
        # Process the query using the RAG service
        logger.info(f"Processing query: '{request.query}'")
        result = rag_service.query(
            query_text=request.query,
            top_k=5,
            similarity_threshold=0.3
        )

        logger.info(f"Retrieved {result.get('retrieved_docs_count', 0)} documents for query: '{request.query}'")

        # Log retrieved document information for debugging
        if result.get('retrieved_docs_count', 0) > 0:
            logger.info(f"Retrieved {result['retrieved_docs_count']} documents:")
            for i, doc in enumerate(result.get('sources', [])[:3]):  # Log first 3 sources
                content_preview = doc.get('content', '')[:200] if isinstance(doc, dict) else str(doc)[:200]
                logger.info(f"  Doc {i+1}: {content_preview}...")
        else:
            logger.warning(f"No documents retrieved for query: '{request.query}'")

        # Check if no documents were retrieved
        if result.get("retrieved_docs_count", 0) == 0:
            logger.warning(f"No relevant content found for query: '{request.query}'")
            response = BookWideQueryResponse(
                answer="ERROR: No book content retrieved. RAG pipeline broken.",
                sources=[],
                session_id=request.session_id,
                confidence=0.0,
                retrieved_docs_count=0
            )
        else:
            # Create response object for valid results
            response = BookWideQueryResponse(
                answer=result["answer"],
                sources=result.get("source_references", []),
                session_id=request.session_id,
                confidence=result.get("confidence", 0.5),
                retrieved_docs_count=result["retrieved_docs_count"]
            )

        return response

    except Exception as e:
        logger.error(f"Error in book_wide_qa: {str(e)}", exc_info=True)
        # Return error message instead of fallback
        error_response = BookWideQueryResponse(
            answer="ERROR: No book content retrieved. RAG pipeline broken.",
            sources=[],
            session_id=request.session_id,
            confidence=0.0,
            retrieved_docs_count=0
        )
        return error_response


@app.post("/api/v1/rag/selected-text", response_model=SelectedTextQueryResponse)
async def selected_text_qa(request: SelectedTextQueryRequest):
    """
    Selected-text Q&A endpoint - Ask questions about user-provided text only (ignores global context).
    """
    try:
        # For selected text, we'll provide a custom response that explains the text
        # In a real implementation, you could still use the vector database to find
        # related content in the book, but for now we'll return a placeholder response
        # that is based on the selected text

        # Create a response based on selected text
        response = SelectedTextQueryResponse(
            answer=f"Based on the selected text you provided: '{request.selected_text[:100]}...', the answer to your query '{request.query}' is: This is a placeholder answer based on the selected text.",
            sources=[{
                "source_type": "selected_text",
                "text_preview": request.selected_text[:100] + ("..." if len(request.selected_text) > 100 else ""),
                "confidence": 0.9
            }],
            session_id=request.session_id,
            mode="selected_text",
            confidence=0.9
        )
        return response

    except Exception as e:
        logger.error(f"Error in selected_text_qa: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )


# Add basic exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    from fastapi.responses import JSONResponse
    logger.error(f"Global exception occurred: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"message": "An internal server error occurred", "error": str(exc)}
    )


if __name__ == "__main__":
    import uvicorn
    logger.info("Starting Physical AI & Humanoid Robotics RAG Chatbot API")
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)