#!/usr/bin/env python3
"""
Script to index the Physical AI & Humanoid Robotics book content into the vector database.
This script loads all markdown files from the docs directory and indexes them.
"""

import os
import sys
import logging
from pathlib import Path

# Add the backend directory to Python path
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

# Add the parent directory to Python path for relative imports
parent_dir = os.path.dirname(backend_dir)
sys.path.insert(0, parent_dir)

# Import the modules
from backend.services.embedding_service import EmbeddingService
from backend.services.retrieval_agent import RetrievalAgent
from backend.services.content_indexing_service import ContentIndexingService
from backend.ingest_books import read_book_content

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def main():
    logger.info("Starting document indexing process...")

    # Initialize services
    logger.info("Initializing embedding service with 512 dimensions...")
    embedding_service = EmbeddingService(expected_dimension=512)

    logger.info("Initializing retrieval agent...")
    retrieval_agent = RetrievalAgent(embedding_service)

    logger.info("Initializing content indexing service...")
    indexing_service = ContentIndexingService(embedding_service, retrieval_agent)

    # Load book content
    docs_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs")
    logger.info(f"Loading book content from: {docs_path}")

    if not os.path.exists(docs_path):
        logger.error(f"Docs directory does not exist: {docs_path}")
        return False

    try:
        # Initialize the Qdrant collection first
        logger.info("Initializing Qdrant collection...")
        retrieval_agent.initialize_collection()

        documents = read_book_content(docs_path)
        logger.info(f"Loaded {len(documents)} document chunks from the book")

        if not documents:
            logger.error("No documents found in the docs directory")
            return False

        # Combine all content for indexing
        logger.info("Combining content for indexing...")
        all_content = []
        for doc in documents:
            all_content.append(doc['content'])

        full_content = "\n\n".join(all_content)

        # Index the content
        logger.info("Starting indexing process...")
        indexing_result = indexing_service.index_book_content(
            book_id="physical_ai_humanoid_robotics_book",
            title="Physical AI & Humanoid Robotics",
            content=full_content,
            author="Book Authors",
            chunk_size=512
        )

        logger.info(f"Indexing completed: {indexing_result}")

        # Verify the indexing by checking vector database
        logger.info("Verifying indexed content in vector database...")

        # Perform a test query to see if content was indexed
        test_results = retrieval_agent.retrieve_similar_chunks(
            query="What is Physical AI?",
            limit=3,
            threshold=0.0  # Very low threshold to see if anything was indexed
        )

        logger.info(f"Test retrieval found {len(test_results)} results")
        for i, result in enumerate(test_results):
            logger.info(f"Result {i+1}: Score={result['relevance_score']:.3f}, Preview='{result['content'][:100]}...'")

        logger.info("Document indexing completed successfully!")
        return True

    except Exception as e:
        logger.error(f"Error during indexing: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    if success:
        logger.info("Indexing process completed successfully!")
        sys.exit(0)
    else:
        logger.error("Indexing process failed!")
        sys.exit(1)