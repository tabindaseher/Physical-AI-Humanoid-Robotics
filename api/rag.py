import logging
from fastapi import APIRouter, Depends, HTTPException
from sqlmodel import Session
from typing import Optional

# Import using relative imports that work with uvicorn
from ..models.request import BookWideQueryRequest, SelectedTextQueryRequest
from ..models.response import BookWideQueryResponse, SelectedTextQueryResponse
from ..config.database import get_session
from ..services.storage_service import StorageService
from ..services.embedding_service import EmbeddingService
from ..services.retrieval_agent import RetrievalAgent
from ..services.answer_generator import AnswerGenerator
from ..services.context_validator import ContextValidator
from ..services.citation_service import CitationService
from ..utils.validation import validate_query, validate_session_id, validate_selected_text
from datetime import datetime
import uuid

logger = logging.getLogger(__name__)

router = APIRouter()

# Initialize services (in a real app, you'd use dependency injection)
# Use 512 dimensions to match the Qdrant collection configuration (Cohere model)
embedding_service = EmbeddingService(expected_dimension=512)
retrieval_agent = RetrievalAgent(embedding_service)
answer_generator = AnswerGenerator(embedding_service)
context_validator = ContextValidator()

# Initialize Qdrant collection
retrieval_agent.initialize_collection()


@router.post("/rag/book-wide", response_model=BookWideQueryResponse)
async def book_wide_qa(request: BookWideQueryRequest, session: Session = Depends(get_session)) -> BookWideQueryResponse:
    """
    Book-wide Q&A endpoint - Ask questions about the entire book content.
    """
    # Validate inputs
    if not validate_query(request.query):
        raise HTTPException(status_code=400, detail="Invalid query provided")

    if not validate_session_id(request.session_id):
        raise HTTPException(status_code=400, detail="Invalid session ID provided")

    try:
        # Get or create session
        storage_service = StorageService(session)
        db_session = storage_service.get_session(request.session_id)

        if not db_session:
            # Create new session
            session_data = {
                "session_id": request.session_id,
                "selected_text": None,
                "query_history": {"queries": []},
                "session_metadata": request.metadata or {}
            }
            db_session = storage_service.create_session(session_data)

        # Retrieve similar content chunks from the book
        # For book-wide search, we don't filter by specific book_id, but in a real system you might want to
        logger.info(f"Starting retrieval for query: '{request.query}'")
        similar_chunks = retrieval_agent.retrieve_similar_chunks(
            query=request.query,
            book_id=None,  # Search across all books (in a real system, you might specify a book_id)
            limit=5,
            threshold=0.6  # Adjust threshold as needed
        )
        logger.info(f"Retrieved {len(similar_chunks)} similar chunks for query: '{request.query}'")

        # Log retrieved content for debugging
        if similar_chunks:
            logger.info(f"Number of indexed documents available: {len(similar_chunks)}")
            for i, chunk in enumerate(similar_chunks):
                logger.info(f"Retrieved document {i+1}: Title='{chunk.get('location', {}).get('title', 'N/A')}', "
                           f"Score={chunk.get('relevance_score', 0):.3f}, "
                           f"Preview='{chunk.get('content', '')[:100]}...'")
        else:
            logger.warning(f"No documents retrieved for query: '{request.query}'")

        if not similar_chunks:
            # No relevant content found - return error message that indicates RAG pipeline is broken
            logger.warning(f"No similar chunks retrieved for query: {request.query}")
            return BookWideQueryResponse(
                answer="ERROR: No book content retrieved. RAG pipeline broken.",
                sources=[],
                session_id=request.session_id,
                confidence=0.0
            )

        # Generate answer based on retrieved chunks
        answer_result = answer_generator.generate_answer(
            query=request.query,
            context_chunks=similar_chunks
        )

        # Validate the response context
        validation_result = context_validator.validate_response_context(
            answer=answer_result["answer"],
            context_chunks=similar_chunks,
            query=request.query
        )

        # If validation fails, return a safe response but still provide the answer if available
        if not validation_result["is_context_valid"]:
            logger.warning(f"Context validation failed for query: {request.query}")
            # Still return the answer if we have one, but with lower confidence
            if answer_result and answer_result.get("answer"):
                return BookWideQueryResponse(
                    answer=answer_result["answer"],
                    sources=answer_result.get("source_references", []),
                    session_id=request.session_id,
                    confidence=0.1  # Low confidence due to validation failure
                )
            else:
                return BookWideQueryResponse(
                    answer="ERROR: No book content retrieved. RAG pipeline broken.",
                    sources=[],
                    session_id=request.session_id,
                    confidence=0.0
                )

        # Create response object
        response = BookWideQueryResponse(
            answer=answer_result["answer"],
            sources=answer_result["source_references"],
            session_id=request.session_id,
            confidence=answer_result.get("confidence", 0.5)
        )

        # Store the query and response in the session
        if db_session.query_history is None:
            db_session.query_history = {"queries": []}

        db_session.query_history["queries"].append({
            "query": request.query,
            "response": response.answer,
            "timestamp": datetime.utcnow().isoformat(),
            "sources": [source for source in response.sources]
        })

        # Update session
        storage_service.update_session(request.session_id, {
            "query_history": db_session.query_history,
            "updated_at": datetime.utcnow()
        })

        # Store the response in the database
        response_data = {
            "session_id": request.session_id,
            "query": request.query,
            "answer": response.answer,
            "source_references": [source for source in response.sources],
            "confidence_score": response.confidence,
            "mode": "book-wide"
        }
        storage_service.create_response(response_data)

        return response

    except Exception as e:
        logger.error(f"Error in book_wide_qa: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/rag/selected-text", response_model=SelectedTextQueryResponse)
async def selected_text_qa(request: SelectedTextQueryRequest, session: Session = Depends(get_session)) -> SelectedTextQueryResponse:
    """
    Selected-text Q&A endpoint - Ask questions about user-provided text only (ignores global context).
    """
    # Validate inputs
    if not validate_query(request.query):
        raise HTTPException(status_code=400, detail="Invalid query provided")

    if not validate_session_id(request.session_id):
        raise HTTPException(status_code=400, detail="Invalid session ID provided")

    if not validate_selected_text(request.selected_text):
        raise HTTPException(status_code=400, detail="Selected text is too long or invalid")

    try:
        # Get or create session
        storage_service = StorageService(session)
        db_session = storage_service.get_session(request.session_id)

        if not db_session:
            # Create new session
            session_data = {
                "session_id": request.session_id,
                "selected_text": request.selected_text,
                "query_history": {"queries": []},
                "session_metadata": request.metadata or {}
            }
            db_session = storage_service.create_session(session_data)

        # For selected-text mode, we only use the provided text as context
        # Create a mock chunk with the selected text
        selected_text_chunk = [{
            "chunk_id": str(uuid.uuid4()),
            "content": request.selected_text,
            "text_preview": request.selected_text[:200] + "..." if len(request.selected_text) > 200 else request.selected_text,
            "relevance_score": 1.0,  # Perfect relevance since this is the exact context
            "location": {"source": "selected_text"},
            "book_id": "selected_text",
            "hash": "selected_text_hash"
        }]

        # Generate answer based only on the selected text
        answer_result = answer_generator.generate_answer(
            query=request.query,
            context_chunks=selected_text_chunk
        )

        # Validate context isolation - ensure answer is based only on selected text
        isolation_validation = context_validator.validate_context_isolation(
            mode="selected-text",
            answer=answer_result["answer"],
            provided_context=request.selected_text,
            global_context_available=True  # Global context exists but shouldn't be used
        )

        if not isolation_validation["context_isolation_valid"]:
            raise HTTPException(status_code=500, detail="Context isolation validation failed")

        # Validate the response context
        validation_result = context_validator.validate_response_context(
            answer=answer_result["answer"],
            context_chunks=selected_text_chunk,
            query=request.query
        )

        # If validation fails, return a safe response
        if not validation_result["is_context_valid"]:
            return SelectedTextQueryResponse(
                answer="I couldn't generate a properly sourced answer based on the selected text.",
                sources=[],
                session_id=request.session_id,
                mode="selected_text",
                confidence=0.0
            )

        # Create response object
        response = SelectedTextQueryResponse(
            answer=answer_result["answer"],
            sources=[{
                "source_type": "selected_text",
                "text_preview": request.selected_text[:100] + "..." if len(request.selected_text) > 100 else request.selected_text,
                "confidence": answer_result.get("confidence", 0.5)
            }],
            session_id=request.session_id,
            mode="selected_text",
            confidence=answer_result.get("confidence", 0.5)
        )

        # Store the query and response in the session
        if db_session.query_history is None:
            db_session.query_history = {"queries": []}

        db_session.query_history["queries"].append({
            "query": request.query,
            "response": response.answer,
            "timestamp": datetime.utcnow().isoformat(),
            "selected_text_used": request.selected_text[:100] + "..."
        })

        # Update session
        storage_service.update_session(request.session_id, {
            "selected_text": request.selected_text,  # Update with latest selected text
            "query_history": db_session.query_history,
            "updated_at": datetime.utcnow()
        })

        # Store the response in the database
        response_data = {
            "session_id": request.session_id,
            "query": request.query,
            "answer": response.answer,
            "source_references": [{"source_type": "selected_text", "text_preview": request.selected_text[:100]}],
            "confidence_score": response.confidence,
            "mode": "selected-text"
        }
        storage_service.create_response(response_data)

        return response

    except Exception as e:
        logger.error(f"Error in selected_text_qa: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")