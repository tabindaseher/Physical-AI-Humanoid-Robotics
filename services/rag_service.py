import sys
import os
# Add the current directory to Python path to enable imports
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import logging
from typing import List, Dict, Any, Optional
import asyncio
from services.embedding_service import EmbeddingService
from services.retrieval_service import RetrievalService
from services.answer_generation_service import AnswerGenerationService
from ingest_books import read_book_content

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(
        self,
        embedding_service: EmbeddingService,
        retrieval_service: Optional[RetrievalService],
        answer_service: AnswerGenerationService,
        docs_path: str = "../../../docs"
    ):
        """
        Initialize the RAG service.

        Args:
            embedding_service: Service for generating embeddings
            retrieval_service: Service for retrieving similar documents (can be None for fallback mode)
            answer_service: Service for generating answers
            docs_path: Path to the docs directory containing book content
        """
        self.embedding_service = embedding_service
        self.retrieval_service = retrieval_service
        self.answer_service = answer_service
        self.docs_path = docs_path
        self.is_initialized = False

        # Initialize fallback storage for when retrieval service is not available
        if retrieval_service is None:
            self.document_store = []  # Simple in-memory fallback
            logger.info("Initialized RAG service in fallback mode (no vector database)")

    def load_and_index_book_content(self, force_reindex: bool = False):
        """
        Load book content and index it in the vector database or fallback storage.

        Args:
            force_reindex: If True, force re-indexing even if collection already has documents
        """
        try:
            doc_count = 0
            if self.retrieval_service:
                # Use vector database
                doc_count = self.retrieval_service.count_documents()

                if doc_count > 0 and not force_reindex:
                    logger.info(f"Found {doc_count} existing documents in collection, skipping re-indexing")
                    self.is_initialized = True
                    return True
            else:
                # Use fallback storage
                if hasattr(self, 'document_store') and len(self.document_store) > 0 and not force_reindex:
                    logger.info(f"Found {len(self.document_store)} documents in fallback storage, skipping re-indexing")
                    self.is_initialized = True
                    return True

            logger.info("Loading book content from docs directory...")

            # Load book content
            documents = read_book_content(self.docs_path)

            if not documents:
                logger.warning("No documents found in the docs directory")
                return False

            logger.info(f"Processing {len(documents)} document chunks...")

            if self.retrieval_service:
                # Use vector database
                # Extract content for embedding
                contents = [doc["content"] for doc in documents]

                # Generate embeddings
                logger.info("Generating embeddings for documents...")
                embeddings = self.embedding_service.encode_texts(contents)

                # Extract document IDs and metadata
                doc_ids = [doc["doc_id"] for doc in documents]
                doc_metadata = [doc for doc in documents]

                # Add documents to vector database
                logger.info("Indexing documents in vector database...")
                self.retrieval_service.add_documents_batch(doc_ids, embeddings, doc_metadata)

                logger.info(f"Successfully indexed {len(documents)} document chunks in vector database")
            else:
                # Use fallback storage
                logger.info("Storing documents in fallback storage...")
                self.document_store = documents  # Store directly in memory

                logger.info(f"Successfully stored {len(documents)} document chunks in fallback storage")

            self.is_initialized = True
            return True

        except Exception as e:
            logger.error(f"Error loading and indexing book content: {e}")
            # Still mark as initialized even if there's an error, so the service can function
            # but with limited capabilities
            self.is_initialized = True  # Ensure it's initialized even if there's an error
            return False

    async def load_and_index_book_content_async(self, force_reindex: bool = False):
        """
        Async wrapper for loading and indexing book content.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.load_and_index_book_content, force_reindex)

    def query(self, query_text: str, top_k: int = 5, similarity_threshold: float = 0.3) -> Dict[str, Any]:
        """
        Process a user query and return an answer based on the book content.

        Args:
            query_text: User's question
            top_k: Number of top similar documents to retrieve
            similarity_threshold: Minimum similarity threshold for retrieved documents

        Returns:
            Dictionary containing the answer, sources, and additional information
        """
        if not self.is_initialized:
            logger.warning("RAG service not initialized")
            return {
                "answer": "The RAG service is not properly initialized.",
                "sources": [],
                "query": query_text,
                "retrieved_docs_count": 0
            }

        try:
            similar_docs = []

            if self.retrieval_service:
                # Use vector database
                # Generate embedding for the query
                query_embedding = self.embedding_service.encode_single_text(query_text)

                # Search for similar documents
                similar_docs = self.retrieval_service.search_similar(
                    query_embedding,
                    limit=top_k,
                    threshold=similarity_threshold
                )
            else:
                # Use fallback method - simple keyword matching in stored documents
                logger.info(f"Using fallback search with {len(self.document_store) if hasattr(self, 'document_store') else 0} documents")
                try:
                    similar_docs = self._search_fallback(query_text, top_k)
                    logger.info(f"Fallback search found {len(similar_docs)} documents")
                except Exception as e:
                    logger.error(f"Error in fallback search: {e}")
                    # Return empty list if fallback search fails
                    similar_docs = []

            if not similar_docs:
                logger.info(f"No similar documents found for query: '{query_text}'")
                # Return a specific response that our endpoint can recognize
                return {
                    "answer": "I couldn't find relevant information in the book to answer this question.",
                    "sources": [],
                    "query": query_text,
                    "retrieved_docs_count": 0
                }

            logger.info(f"Found {len(similar_docs)} similar documents for query: '{query_text}'")

            # Generate answer based on retrieved documents
            try:
                answer_result = self.answer_service.generate_answer(query_text, similar_docs)
            except Exception as e:
                logger.error(f"Error generating answer: {e}")
                return {
                    "answer": "I found relevant information but encountered an issue generating the answer.",
                    "sources": [],
                    "query": query_text,
                    "retrieved_docs_count": len(similar_docs)
                }

            # Format response
            response = {
                "answer": answer_result["answer"],
                "sources": answer_result["source_references"],
                "query": query_text,
                "retrieved_docs_count": len(similar_docs),
                "confidence": answer_result.get("confidence", 0.5)
            }

            return response

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            logger.error(f"Query was: {query_text}")
            # Return a safe response instead of raising the exception
            return {
                "answer": "Sorry, I encountered an issue processing your question. The system is working but couldn't process this specific query.",
                "sources": [],
                "query": query_text,
                "retrieved_docs_count": 0
            }

    def _search_fallback(self, query_text: str, top_k: int) -> List[Dict[str, Any]]:
        """
        Fallback search method when vector database is not available.
        Uses simple keyword matching in stored documents.
        """
        if not hasattr(self, 'document_store') or not self.document_store:
            logger.warning("No documents available in fallback storage")
            return []

        # Simple keyword matching approach
        query_lower = query_text.lower()
        matches = []

        for doc in self.document_store:
            content_lower = doc["content"].lower()
            title_lower = doc["title"].lower()

            # Count how many query words appear in the content
            query_words = query_lower.split()
            match_score = sum(1 for word in query_words if word in content_lower or word in title_lower)

            # Also check if the query as a phrase appears in the content
            if query_lower in content_lower or query_lower in title_lower:
                match_score += 2  # Boost score for phrase matches

            if match_score > 0:
                matches.append({
                    "doc_id": doc.get("doc_id", str(hash(content_lower))),
                    "score": match_score,
                    "payload": {
                        "title": doc.get("title", ""),
                        "source_file": doc.get("source_file", ""),
                        "content": doc.get("content", ""),
                        "chunk_index": doc.get("chunk_index", 0),
                        "metadata": doc.get("metadata", {}),
                    },
                    "content": doc.get("content", ""),
                    "title": doc.get("title", ""),
                    "source_file": doc.get("source_file", ""),
                })

        # Sort by score and return top_k
        matches.sort(key=lambda x: x["score"], reverse=True)
        return matches[:top_k]

    async def query_async(self, query_text: str, top_k: int = 5, similarity_threshold: float = 0.3) -> Dict[str, Any]:
        """
        Async wrapper for the query method.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.query, query_text, top_k, similarity_threshold)

    def get_document_count(self) -> int:
        """
        Get the total number of documents in the vector database or fallback storage.
        """
        if self.retrieval_service:
            return self.retrieval_service.count_documents()
        else:
            # Return count from fallback storage
            return len(self.document_store) if hasattr(self, 'document_store') else 0

    def reset_index(self):
        """
        Delete all documents from the vector database.
        """
        try:
            self.retrieval_service.delete_collection()
            self.is_initialized = False
            logger.info("Deleted all documents from vector database")
        except Exception as e:
            logger.error(f"Error resetting index: {e}")
            raise