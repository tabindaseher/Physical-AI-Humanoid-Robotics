import sys
import os
# Add the current directory to Python path to enable imports
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import logging
from typing import List, Dict, Any, Optional
import uuid
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import numpy as np
from models.response import SourceReference

logger = logging.getLogger(__name__)

class RetrievalService:
    def __init__(
        self,
        host: str = "localhost",
        port: int = 6333,
        collection_name: str = "book_embeddings",
        vector_size: int = 384  # Default size for all-MiniLM-L6-v2
    ):
        """
        Initialize the retrieval service.
        
        Args:
            host: Qdrant host address
            port: Qdrant port
            collection_name: Name of the collection to store embeddings
            vector_size: Size of the embedding vectors
        """
        try:
            # Initialize Qdrant client
            self.client = QdrantClient(host=host, port=port)
            self.collection_name = collection_name
            self.vector_size = vector_size
            
            # Create collection if it doesn't exist
            self._create_collection()
            
            logger.info(f"Connected to Qdrant collection: {collection_name}")
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}")
            raise

    def _create_collection(self):
        """Create the collection if it doesn't exist."""
        try:
            collections = self.client.get_collections()
            
            if self.collection_name not in [coll.name for coll in collections.collections]:
                # Create a new collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=self.vector_size, distance=Distance.COSINE),
                )
                
                logger.info(f"Created collection '{self.collection_name}' with {self.vector_size}-dimensional vectors")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists")
        except Exception as e:
            logger.error(f"Error creating collection: {e}")
            raise

    def add_document(self, doc_id: str, embedding: List[float], document_data: Dict[str, Any]):
        """
        Add a document to the vector database.
        
        Args:
            doc_id: Unique document identifier
            embedding: Embedding vector
            document_data: Metadata for the document
        """
        try:
            points = [
                PointStruct(
                    id=doc_id,
                    vector=embedding,
                    payload={
                        "title": document_data.get("title", ""),
                        "source_file": document_data.get("source_file", ""),
                        "content": document_data.get("content", ""),
                        "chunk_index": document_data.get("chunk_index", 0),
                        "metadata": document_data.get("metadata", {}),
                    }
                )
            ]
            
            self.client.upsert(collection_name=self.collection_name, points=points)
            
            logger.debug(f"Added document to collection: {doc_id}")
        except Exception as e:
            logger.error(f"Error adding document {doc_id} to collection: {e}")
            raise

    def add_documents_batch(self, doc_ids: List[str], embeddings: np.ndarray, documents_data: List[Dict[str, Any]]):
        """
        Add multiple documents to the vector database in batch.
        
        Args:
            doc_ids: List of document identifiers
            embeddings: Array of embedding vectors
            documents_data: List of metadata dictionaries
        """
        try:
            points = []
            
            for doc_id, embedding, doc_data in zip(doc_ids, embeddings, documents_data):
                point = PointStruct(
                    id=doc_id,
                    vector=embedding.tolist() if isinstance(embedding, np.ndarray) else embedding,
                    payload={
                        "title": doc_data.get("title", ""),
                        "source_file": doc_data.get("source_file", ""),
                        "content": doc_data.get("content", ""),
                        "chunk_index": doc_data.get("chunk_index", 0),
                        "metadata": doc_data.get("metadata", {}),
                    }
                )
                points.append(point)
            
            self.client.upsert(collection_name=self.collection_name, points=points)
            
            logger.info(f"Added {len(points)} documents to collection in batch")
        except Exception as e:
            logger.error(f"Error adding documents batch to collection: {e}")
            raise

    def search_similar(self, query_embedding: List[float], limit: int = 5, threshold: float = 0.3) -> List[Dict[str, Any]]:
        """
        Search for similar documents to the query embedding.
        
        Args:
            query_embedding: Query embedding vector
            limit: Maximum number of results to return
            threshold: Minimum similarity score threshold
        
        Returns:
            List of similar documents with their scores
        """
        try:
            # Search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                score_threshold=threshold,
            )
            
            results = []
            for hit in search_results:
                result = {
                    "doc_id": hit.id,
                    "score": hit.score,
                    "payload": hit.payload,
                    "content": hit.payload.get("content", ""),
                    "title": hit.payload.get("title", ""),
                    "source_file": hit.payload.get("source_file", ""),
                }
                results.append(result)
            
            logger.debug(f"Found {len(results)} similar documents")
            return results
            
        except Exception as e:
            logger.error(f"Error searching for similar documents: {e}")
            raise

    def get_all_documents(self) -> List[Dict[str, Any]]:
        """
        Retrieve all documents from the collection.
        
        Returns:
            List of all documents
        """
        try:
            records, _ = self.client.scroll(
                collection_name=self.collection_name,
                limit=10000  # Adjust as needed
            )
            
            documents = []
            for record in records:
                document = {
                    "doc_id": record.id,
                    "payload": record.payload,
                    "content": record.payload.get("content", ""),
                    "title": record.payload.get("title", ""),
                    "source_file": record.payload.get("source_file", ""),
                }
                documents.append(document)
            
            return documents
        except Exception as e:
            logger.error(f"Error retrieving all documents: {e}")
            raise

    def delete_collection(self):
        """Delete the entire collection."""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error deleting collection: {e}")
            raise

    def count_documents(self) -> int:
        """Get the total number of documents in the collection."""
        try:
            collection_info = self.client.get_collection(collection_name=self.collection_name)
            return collection_info.points_count
        except Exception as e:
            logger.error(f"Error counting documents: {e}")
            return 0