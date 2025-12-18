from sqlmodel import SQLModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid
from sqlalchemy import Column
from sqlalchemy.dialects.postgresql import JSONB

# -----------------------------
# QuestionSession Table
# -----------------------------
class QuestionSession(SQLModel, table=True):
    session_id: str = Field(default_factory=lambda: str(uuid.uuid4()), primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    selected_text: Optional[str] = Field(default=None, max_length=10000)  # User-provided text
    query_history: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSONB)
    )
    session_metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSONB)
    )

# -----------------------------
# BookContentChunk Table
# -----------------------------
class BookContentChunk(SQLModel, table=True):
    chunk_id: str = Field(default_factory=lambda: str(uuid.uuid4()), primary_key=True)
    book_id: str
    content: str = Field(min_length=1)
    chunk_metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSONB)
    )
    embedding_vector: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSONB)
    )
    hash: str = Field(max_length=255)  # Content hash for deduplication
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        schema_extra = {
            "example": {
                "book_id": "book_123",
                "content": "This is a sample content chunk from the book...",
                "chunk_metadata": {"chapter": "1", "section": "1.1", "page": 5},
                "hash": "unique_content_hash"
            }
        }

# -----------------------------
# Response Table
# -----------------------------
class Response(SQLModel, table=True):
    response_id: str = Field(default_factory=lambda: str(uuid.uuid4()), primary_key=True)
    session_id: str = Field(foreign_key="questionsession.session_id")
    query: str = Field(min_length=1)
    answer: str = Field(min_length=1)
    source_references: Optional[List[Dict[str, Any]]] = Field(
        default=None,
        sa_column=Column(JSONB)
    )
    confidence_score: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    mode: str = Field(default="book-wide")  # "book-wide" or "selected-text"

    class Config:
        schema_extra = {
            "example": {
                "session_id": "sess_12345",
                "query": "What are the main concepts?",
                "answer": "The main concepts include...",
                "confidence_score": 0.85,
                "mode": "book-wide"
            }
        }

# -----------------------------
# BookMetadata Table
# -----------------------------
class BookMetadata(SQLModel, table=True):
    book_id: str = Field(max_length=255, primary_key=True)
    title: str = Field(min_length=1)
    author: Optional[str] = None
    total_chunks: int = Field(default=0, ge=0)
    indexed_at: Optional[datetime] = Field(default=None)
    session_metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSONB)
    )

    class Config:
        schema_extra = {
            "example": {
                "book_id": "book_123",
                "title": "Physical AI & Humanoid Robotics",
                "author": "Author Name",
                "total_chunks": 150
            }
        }
