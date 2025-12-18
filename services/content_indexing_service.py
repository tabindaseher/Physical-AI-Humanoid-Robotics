from typing import List, Dict, Any
from backend.models.entities import BookContentChunk, BookMetadata
from backend.services.embedding_service import EmbeddingService
from backend.services.retrieval_agent import RetrievalAgent
from backend.utils.chunking import chunk_text_by_semantic_boundaries, validate_chunk_metadata
from sqlmodel import Session
from backend.config.database import get_session
from backend.services.storage_service import StorageService
import hashlib
from datetime import datetime


class ContentIndexingService:
    def __init__(self, embedding_service: EmbeddingService, retrieval_agent: RetrievalAgent):
        self.embedding_service = embedding_service
        self.retrieval_agent = retrieval_agent

    def index_book_content(self, book_id: str, title: str, content: str, author: str = None,
                          chunk_size: int = 512, session: Session = None) -> Dict[str, Any]:
        """
        Index book content by chunking, embedding, and storing in vector database.

        Args:
            book_id: Unique identifier for the book
            title: Title of the book
            content: Full content of the book
            author: Author of the book (optional)
            chunk_size: Size of chunks for semantic chunking
            session: Database session for storing metadata

        Returns:
            Dictionary with indexing results
        """
        # Chunk the content
        chunks = chunk_text_by_semantic_boundaries(content, max_chunk_size=chunk_size)

        # Validate chunks
        valid_chunks = []
        invalid_chunks = []
        for chunk in chunks:
            if validate_chunk_metadata(chunk):
                valid_chunks.append(chunk)
            else:
                invalid_chunks.append(chunk)

        if invalid_chunks:
            print(f"Warning: {len(invalid_chunks)} chunks were invalid and skipped")

        # Create embeddings for valid chunks
        text_list = [chunk["content"] for chunk in valid_chunks]
        embeddings = self.embedding_service.create_embeddings(text_list)

        # Store chunks in vector database and database
        stored_count = 0
        storage_service = StorageService(session) if session else None

        for i, chunk in enumerate(valid_chunks):
            # Create BookContentChunk entity
            chunk_entity = BookContentChunk(
                book_id=book_id,
                content=chunk["content"],
                chunk_metadata=chunk["metadata"],
                hash=chunk["metadata"]["hash"],
                embedding_vector=None  # We'll store the embedding in the vector DB, not in SQL
            )

            # Store in vector database
            self.retrieval_agent.store_chunk(chunk_entity, embeddings[i])

            # Store in SQL database if session provided
            if storage_service:
                try:
                    storage_service.create_book_content_chunk({
                        "book_id": book_id,
                        "content": chunk["content"],
                        "chunk_metadata": chunk["metadata"],
                        "hash": chunk["metadata"]["hash"],
                        "created_at": datetime.utcnow()
                    })
                    stored_count += 1
                except Exception as e:
                    print(f"Error storing chunk in SQL database: {e}")

        # Create/update book metadata
        if storage_service:
            metadata_entity = {
                "book_id": book_id,
                "title": title,
                "author": author,
                "total_chunks": len(valid_chunks),
                "indexed_at": datetime.utcnow(),
                "metadata": {
                    "chunk_size": chunk_size,
                    "total_original_chars": len(content)
                }
            }
            storage_service.create_book_metadata(metadata_entity)

        return {
            "book_id": book_id,
            "total_chunks_processed": len(chunks),
            "valid_chunks": len(valid_chunks),
            "invalid_chunks": len(invalid_chunks),
            "stored_in_vector_db": len(valid_chunks),
            "stored_in_sql": stored_count,
            "indexing_completed_at": datetime.utcnow().isoformat()
        }

    def index_content_chunks(self, book_id: str, chunks: List[Dict[str, Any]], session: Session = None) -> Dict[str, Any]:
        """
        Index pre-chunked content.

        Args:
            book_id: Unique identifier for the book
            chunks: List of pre-chunked content with metadata
            session: Database session for storing metadata

        Returns:
            Dictionary with indexing results
        """
        # Validate chunks
        valid_chunks = []
        invalid_chunks = []
        for chunk in chunks:
            if validate_chunk_metadata(chunk):
                valid_chunks.append(chunk)
            else:
                invalid_chunks.append(chunk)

        # Create embeddings for valid chunks
        text_list = [chunk["content"] for chunk in valid_chunks]
        embeddings = self.embedding_service.create_embeddings(text_list)

        # Store chunks in vector database and database
        stored_count = 0
        storage_service = StorageService(session) if session else None

        for i, chunk in enumerate(valid_chunks):
            # Create BookContentChunk entity
            chunk_entity = BookContentChunk(
                book_id=book_id,
                content=chunk["content"],
                chunk_metadata=chunk["metadata"],
                hash=chunk["metadata"]["hash"],
                embedding_vector=None  # We'll store the embedding in the vector DB, not in SQL
            )

            # Store in vector database
            self.retrieval_agent.store_chunk(chunk_entity, embeddings[i])

            # Store in SQL database if session provided
            if storage_service:
                try:
                    storage_service.create_book_content_chunk({
                        "book_id": book_id,
                        "content": chunk["content"],
                        "chunk_metadata": chunk["metadata"],
                        "hash": chunk["metadata"]["hash"],
                        "created_at": datetime.utcnow()
                    })
                    stored_count += 1
                except Exception as e:
                    print(f"Error storing chunk in SQL database: {e}")

        # Update book metadata
        if storage_service:
            # Get existing metadata or create new
            existing_metadata = storage_service.get_book_metadata(book_id)
            if existing_metadata:
                existing_metadata.total_chunks += len(valid_chunks)
                existing_metadata.indexed_at = datetime.utcnow()
                storage_service.create_book_metadata({
                    "book_id": existing_metadata.book_id,
                    "title": existing_metadata.title,
                    "author": existing_metadata.author,
                    "total_chunks": existing_metadata.total_chunks,
                    "indexed_at": existing_metadata.indexed_at,
                    "metadata": existing_metadata.metadata
                })
            else:
                # Create new metadata record
                metadata_entity = {
                    "book_id": book_id,
                    "title": f"Book {book_id}",
                    "author": "Unknown",
                    "total_chunks": len(valid_chunks),
                    "indexed_at": datetime.utcnow(),
                    "metadata": {"source": "indexed_chunks"}
                }
                storage_service.create_book_metadata(metadata_entity)

        return {
            "book_id": book_id,
            "total_chunks_processed": len(chunks),
            "valid_chunks": len(valid_chunks),
            "invalid_chunks": len(invalid_chunks),
            "stored_in_vector_db": len(valid_chunks),
            "stored_in_sql": stored_count,
            "indexing_completed_at": datetime.utcnow().isoformat()
        }

    def update_content_index(self, book_id: str, new_content: str, session: Session = None) -> Dict[str, Any]:
        """
        Update the index for an existing book by replacing all content.

        Args:
            book_id: Unique identifier for the book to update
            new_content: New content to index
            session: Database session for storing metadata

        Returns:
            Dictionary with update results
        """
        # First, remove existing chunks from vector database
        self.retrieval_agent.delete_chunks_by_book_id(book_id)

        # If session provided, remove from SQL database
        storage_service = StorageService(session) if session else None
        if storage_service:
            # In a real implementation, you would have a method to delete all chunks for a book
            # For now, we'll just update the metadata
            existing_metadata = storage_service.get_book_metadata(book_id)
            if existing_metadata:
                # We'll re-index by calling index_book_content
                pass

        # Re-index the content
        return self.index_book_content(book_id, existing_metadata.title if existing_metadata else f"Book {book_id}",
                                     new_content, existing_metadata.author if existing_metadata else None,
                                     session=session)