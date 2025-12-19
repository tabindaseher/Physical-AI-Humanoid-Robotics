#!/usr/bin/env python3
"""
Direct test of the RAG functionality without requiring a running server
"""

import sys
import os
import logging
import uuid

# Add the backend directory to Python path
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_rag_directly():
    print("Testing RAG functionality directly...")

    # Import the services
    from backend.services.embedding_service import EmbeddingService
    from backend.services.retrieval_agent import RetrievalAgent
    from backend.services.answer_generator import AnswerGenerator

    try:
        # Initialize services
        logger.info("Initializing embedding service with 512 dimensions...")
        embedding_service = EmbeddingService(expected_dimension=512)

        logger.info("Initializing retrieval agent...")
        retrieval_agent = RetrievalAgent(embedding_service)

        logger.info("Initializing answer generator...")
        answer_generator = AnswerGenerator(embedding_service)

        # Initialize the collection (this will create it in memory)
        logger.info("Initializing Qdrant collection...")
        retrieval_agent.initialize_collection()

        # Test retrieval with a query
        query = "What is Physical AI?"
        logger.info(f"Testing retrieval for query: '{query}'")

        results = retrieval_agent.retrieve_similar_chunks(
            query=query,
            limit=5,
            threshold=0.0  # Very low threshold to see if anything was indexed
        )

        logger.info(f"Retrieved {len(results)} results for query: '{query}'")

        for i, result in enumerate(results):
            logger.info(f"Result {i+1}: Score={result['relevance_score']:.3f}")
            logger.info(f"  Preview: {result['content'][:200]}...")

        if len(results) == 0:
            print("ERROR: No book content retrieved. RAG pipeline broken.")
            return False

        # Test answer generation
        logger.info("Testing answer generation...")
        answer_result = answer_generator.generate_answer(
            query=query,
            context_chunks=results
        )

        print(f"Query: {query}")
        print(f"Answer: {answer_result['answer']}")
        print(f"Confidence: {answer_result.get('confidence', 'N/A')}")
        print(f"Sources: {len(answer_result.get('source_references', []))} references")

        return True

    except Exception as e:
        logger.error(f"Error testing RAG functionality: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_rag_directly()
    if success:
        print("\n✅ RAG functionality test completed successfully!")
    else:
        print("\n❌ RAG functionality test failed!")
        sys.exit(1)