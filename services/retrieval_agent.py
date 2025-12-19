from typing import List, Dict, Any, Optional
from .embedding_service import EmbeddingService
from ..models.entities import BookContentChunk
import qdrant_client
from qdrant_client.http import models
from ..config.settings import settings
from ..utils.validation import validate_query
import logging

logger = logging.getLogger(__name__)


class RetrievalAgent:
    def __init__(self, embedding_service: EmbeddingService):
        self.embedding_service = embedding_service
        try:
            # Try to initialize Qdrant client with remote settings
            # If settings.qdrant_url is localhost or empty, use local mode
            if settings.qdrant_url and "localhost" not in settings.qdrant_url and "127.0.0.1" not in settings.qdrant_url:
                self.qdrant_client = qdrant_client.QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key,
                )
                logger.info(f"Connected to remote Qdrant: {settings.qdrant_url}")
            else:
                # Use local mode if URL is localhost or not provided
                # For local mode, we need to specify a path or use in-memory with local=True
                self.qdrant_client = qdrant_client.QdrantClient(
                    path=":memory:"  # Use in-memory storage
                )
                logger.info("Using in-memory Qdrant mode")
        except Exception as e:
            # Fallback to local in-memory mode if remote connection fails
            logger.warning(f"Failed to connect to remote Qdrant: {e}. Using in-memory mode.")
            self.qdrant_client = qdrant_client.QdrantClient(path=":memory:")

        # Define collection name for book content
        self.collection_name = "book_content_chunks"

    def initialize_collection(self):
        """
        Initialize the Qdrant collection for storing book content chunks.
        """
        try:
            # Check if collection exists
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} exists with {collection_info.points_count} points")
        except Exception as e:
            logger.warning(f"Collection doesn't exist or Qdrant is not available: {e}")
            try:
                # Create collection if it doesn't exist
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=512,  # Cohere's embed-multilingual-v3.0 uses 512 dimensions
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created collection {self.collection_name}")
            except Exception as create_error:
                logger.error(f"Failed to create Qdrant collection: {create_error}")
                # Still continue but with reduced functionality

    def store_chunk(self, chunk: BookContentChunk, embedding: List[float]):
        """
        Store a book content chunk with its embedding in Qdrant.

        Args:
            chunk: The BookContentChunk entity
            embedding: The embedding vector for the chunk
        """
        try:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=chunk.chunk_id,
                        vector=embedding,
                        payload={
                            "book_id": chunk.book_id,
                            "content": chunk.content,
                            "chunk_metadata": chunk.chunk_metadata,
                            "hash": chunk.hash
                        }
                    )
                ]
            )
        except Exception as e:
            logger.error(f"Failed to store chunk in Qdrant: {e}")
            # Still continue but with reduced functionality
            raise

    def retrieve_similar_chunks(self, query: str, book_id: Optional[str] = None, limit: int = 5, threshold: float = 0.7) -> List[Dict[str, Any]]:
        """
        Retrieve similar content chunks based on the query.

        Args:
            query: The query to search for
            book_id: Optional book ID to filter results to specific book
            limit: Maximum number of results to return
            threshold: Similarity threshold (0.0-1.0) for filtering results

        Returns:
            List of similar chunks with similarity scores
        """
        if not validate_query(query):
            raise ValueError("Invalid query provided")

        try:
            # Create embedding for the query
            query_embedding = self.embedding_service.create_query_embedding(query)

            # Prepare filters
            filters = []
            if book_id:
                filters.append(models.FieldCondition(
                    key="book_id",
                    match=models.MatchValue(value=book_id)
                ))

            # Perform search - handle both remote and local Qdrant clients
            try:
                # Try the standard search method first (works with remote Qdrant)
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    query_filter=models.Filter(
                        must=filters
                    ) if filters else None,
                    limit=limit,
                    score_threshold=threshold  # Minimum similarity score
                )
            except AttributeError:
                # For local/in-memory Qdrant, use scroll and calculate similarity manually
                logger.warning("Using fallback search method for local Qdrant")
                # Get all points and do basic similarity matching
                try:
                    result = self.qdrant_client.scroll(
                        collection_name=self.collection_name,
                        limit=10000
                    )

                    # The scroll method might return different format depending on client type
                    if isinstance(result, tuple):
                        all_points, next_page = result
                    else:
                        all_points = result

                    if all_points:  # Check if not None or empty
                        # Calculate similarity manually for each point
                        scored_results = []
                        for point in all_points:
                            # Simple cosine similarity calculation
                            # For local Qdrant, the vector might be stored differently
                            point_vector = point.vector if hasattr(point, 'vector') else None
                            if point_vector is not None:
                                similarity = self._calculate_similarity(query_embedding, point_vector)
                                if similarity >= threshold:
                                    # Create a mock search result object similar to the search() result
                                    mock_result = type('MockResult', (), {})()
                                    mock_result.id = point.id
                                    mock_result.score = similarity
                                    mock_result.payload = point.payload
                                    scored_results.append(mock_result)

                        # Sort by score and take top-k
                        scored_results.sort(key=lambda x: x.score, reverse=True)
                        search_results = scored_results[:limit]
                    else:
                        logger.warning("No points found in collection for fallback search")
                        search_results = []
                except Exception as local_search_error:
                    logger.error(f"Error in fallback search: {local_search_error}")
                    search_results = []

            # Format results
            results = []
            for hit in search_results:
                results.append({
                    "chunk_id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "text_preview": hit.payload.get("content", "")[:200] + "..." if len(hit.payload.get("content", "")) > 200 else hit.payload.get("content", ""),
                    "relevance_score": hit.score,
                    "location": hit.payload.get("chunk_metadata", {}),
                    "book_id": hit.payload.get("book_id", ""),
                    "hash": hit.payload.get("hash", "")
                })

            return results
        except Exception as e:
            logger.error(f"Failed to retrieve similar chunks from Qdrant: {e}")
            # Return empty list when Qdrant is not available
            return []

    def retrieve_chunks_by_ids(self, chunk_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve specific chunks by their IDs.

        Args:
            chunk_ids: List of chunk IDs to retrieve

        Returns:
            List of chunks with their information
        """
        points = self.qdrant_client.retrieve(
            collection_name=self.collection_name,
            ids=chunk_ids,
            with_payload=True,
            with_vectors=False
        )

        results = []
        for point in points:
            results.append({
                "chunk_id": point.id,
                "content": point.payload.get("content", ""),
                "location": point.payload.get("chunk_metadata", {}),
                "book_id": point.payload.get("book_id", ""),
                "hash": point.payload.get("hash", "")
            })

        return results

    def process_selected_text_query(self, query: str, selected_text: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Process a query specifically against user-provided selected text only.
        This method bypasses the vector database and works directly with the provided text.

        Args:
            query: The query to process
            selected_text: The user-provided text to search within
            limit: Maximum number of results (not really applicable here since we have only one text)

        Returns:
            List of results based on the selected text
        """
        # Create embedding for the query
        query_embedding = self.embedding_service.create_query_embedding(query)

        # Create embedding for the selected text
        text_embedding = self.embedding_service.create_embeddings([selected_text])[0]

        # Calculate similarity between query and selected text
        # This is a simplified similarity calculation
        similarity_score = self._calculate_similarity(query_embedding, text_embedding)

        # Return the selected text as the result if similarity is above threshold
        if similarity_score > 0.1:  # Low threshold since it's the exact text
            return [{
                "chunk_id": "selected_text",
                "content": selected_text,
                "text_preview": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                "relevance_score": similarity_score,
                "location": {"source": "selected_text"},
                "book_id": "selected_text",
                "hash": "selected_text_hash"
            }]
        else:
            return []  # No relevant content found in the selected text

    def _calculate_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors.

        Args:
            vec1: First vector
            vec2: Second vector

        Returns:
            Similarity score between 0 and 1
        """
        import math

        # Calculate dot product
        dot_product = sum(a * b for a, b in zip(vec1, vec2))

        # Calculate magnitudes
        magnitude1 = math.sqrt(sum(a * a for a in vec1))
        magnitude2 = math.sqrt(sum(b * b for b in vec2))

        # Calculate cosine similarity
        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        similarity = dot_product / (magnitude1 * magnitude2)
        # Normalize to 0-1 range (cosine similarity can be -1 to 1)
        return (similarity + 1) / 2

    def ensure_context_isolation(self, query: str, selected_text: str, search_globally: bool = False) -> List[Dict[str, Any]]:
        """
        Ensure context isolation by only searching within the provided selected text
        and preventing any global knowledge base queries.

        Args:
            query: The query to process
            selected_text: The user-provided text that should be the only context
            search_globally: If True, would allow global search (but this method enforces isolation)

        Returns:
            List of results from only the selected text (enforcing isolation)
        """
        if search_globally:
            # Even if search_globally is True, we enforce isolation for selected-text mode
            print("WARNING: Global search was requested in selected-text mode. Enforcing context isolation.")

        # Process using only the selected text (no global search)
        return self.process_selected_text_query(query, selected_text)

    def delete_chunks_by_book_id(self, book_id: str):
        """
        Delete all chunks associated with a specific book ID.

        Args:
            book_id: The book ID to delete chunks for
        """
        self.qdrant_client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="book_id",
                            match=models.MatchValue(value=book_id)
                        )
                    ]
                )
            )
        )