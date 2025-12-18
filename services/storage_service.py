from sqlmodel import Session, select
from typing import List, Optional, Dict, Any
from backend.models.entities import QuestionSession, BookContentChunk, Response, BookMetadata
from backend.config.database import get_session


class StorageService:
    def __init__(self, session: Session):
        self.session = session

    def create_session(self, session_data: Dict[str, Any]) -> QuestionSession:
        """Create a new question session."""
        db_session = QuestionSession(**session_data)
        self.session.add(db_session)
        self.session.commit()
        self.session.refresh(db_session)
        return db_session

    def get_session(self, session_id: str) -> Optional[QuestionSession]:
        """Get a question session by ID."""
        statement = select(QuestionSession).where(QuestionSession.session_id == session_id)
        return self.session.exec(statement).first()

    def update_session(self, session_id: str, update_data: Dict[str, Any]) -> Optional[QuestionSession]:
        """Update a question session."""
        db_session = self.get_session(session_id)
        if db_session:
            for key, value in update_data.items():
                setattr(db_session, key, value)
            self.session.add(db_session)
            self.session.commit()
            self.session.refresh(db_session)
        return db_session

    def create_book_content_chunk(self, chunk_data: Dict[str, Any]) -> BookContentChunk:
        """Create a new book content chunk."""
        db_chunk = BookContentChunk(**chunk_data)
        self.session.add(db_chunk)
        self.session.commit()
        self.session.refresh(db_chunk)
        return db_chunk

    def get_book_content_chunks(self, book_id: str, limit: int = 100) -> List[BookContentChunk]:
        """Get book content chunks for a specific book."""
        statement = select(BookContentChunk).where(BookContentChunk.book_id == book_id).limit(limit)
        return self.session.exec(statement).all()

    def get_book_content_chunk_by_hash(self, book_id: str, content_hash: str) -> Optional[BookContentChunk]:
        """Get a specific book content chunk by book ID and hash."""
        statement = select(BookContentChunk).where(
            BookContentChunk.book_id == book_id,
            BookContentChunk.hash == content_hash
        )
        return self.session.exec(statement).first()

    def create_response(self, response_data: Dict[str, Any]) -> Response:
        """Create a new response record."""
        db_response = Response(**response_data)
        self.session.add(db_response)
        self.session.commit()
        self.session.refresh(db_response)
        return db_response

    def get_responses_by_session(self, session_id: str) -> List[Response]:
        """Get all responses for a specific session."""
        statement = select(Response).where(Response.session_id == session_id)
        return self.session.exec(statement).all()

    def create_book_metadata(self, metadata_data: Dict[str, Any]) -> BookMetadata:
        """Create or update book metadata."""
        # Check if book already exists
        existing = self.get_book_metadata(metadata_data["book_id"])
        if existing:
            # Update existing
            for key, value in metadata_data.items():
                setattr(existing, key, value)
            self.session.add(existing)
            self.session.commit()
            self.session.refresh(existing)
            return existing
        else:
            # Create new
            db_metadata = BookMetadata(**metadata_data)
            self.session.add(db_metadata)
            self.session.commit()
            self.session.refresh(db_metadata)
            return db_metadata

    def get_book_metadata(self, book_id: str) -> Optional[BookMetadata]:
        """Get book metadata by book ID."""
        statement = select(BookMetadata).where(BookMetadata.book_id == book_id)
        return self.session.exec(statement).first()

    def get_all_books_metadata(self) -> List[BookMetadata]:
        """Get metadata for all books."""
        statement = select(BookMetadata)
        return self.session.exec(statement).all()