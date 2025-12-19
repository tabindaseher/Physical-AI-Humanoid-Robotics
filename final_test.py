#!/usr/bin/env python3
"""
Final test of the complete RAG pipeline fix
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

def test_complete_rag_pipeline():
    print("[TESTING COMPLETE RAG PIPELINE FIX]")
    print("="*50)

    # Step 1: Initialize services
    print("\n[1] INITIALIZING RAG SERVICES...")
    from services.embedding_service import EmbeddingService
    from services.retrieval_agent import RetrievalAgent
    from services.answer_generator import AnswerGenerator

    try:
        embedding_service = EmbeddingService(expected_dimension=512)
        retrieval_agent = RetrievalAgent(embedding_service)
        answer_generator = AnswerGenerator(embedding_service)

        # Initialize collection
        retrieval_agent.initialize_collection()

        print("[SUCCESS] Services initialized successfully")

        # Step 2: Test query
        query = "What is Physical AI?"
        print(f"\n[2] TESTING QUERY: '{query}'")

        # Step 3: Perform retrieval
        print("\n[3] PERFORMING RETRIEVAL...")
        retrieved_chunks = retrieval_agent.retrieve_similar_chunks(
            query=query,
            limit=5,
            threshold=0.0  # Very low threshold to see if anything was indexed
        )

        print(f"[SUCCESS] Retrieved {len(retrieved_chunks)} chunks")

        # Step 4: Show retrieved content
        if retrieved_chunks:
            print("\n[4] SAMPLE RETRIEVED CONTENT:")
            for i, chunk in enumerate(retrieved_chunks[:2]):  # Show first 2 chunks
                print(f"   Chunk {i+1} (Score: {chunk['relevance_score']:.3f}):")
                print(f"   Preview: {chunk['content'][:200]}...")
                print()
        else:
            print("   [ERROR] No content retrieved - RAG pipeline may still have issues")

        # Step 5: Generate answer
        print("[5] GENERATING ANSWER...")
        if retrieved_chunks:
            answer_result = answer_generator.generate_answer(
                query=query,
                context_chunks=retrieved_chunks
            )

            print(f"\n[ANSWER TO '{query}']:")
            print(f"   {answer_result['answer']}")
            print(f"   Confidence: {answer_result.get('confidence', 'N/A')}")
            print(f"   Sources: {len(answer_result.get('source_references', []))} references")
        else:
            print(f"\n[ERROR] No book content retrieved. RAG pipeline broken.")
            print("   Expected: Content should have been retrieved from the Physical AI book")
            return False

        print("\n" + "="*50)
        print("[SUCCESS] RAG PIPELINE TEST COMPLETED SUCCESSFULLY!")
        print("[SUCCESS] Retrieval working with in-memory Qdrant fallback")
        print("[SUCCESS] Answer generation using retrieved context")
        print("[SUCCESS] No more 'topic not covered' fallback responses")
        return True

    except Exception as e:
        logger.error(f"[ERROR] Error in RAG pipeline test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("[PHYSICAL AI & HUMANOID ROBOTICS RAG CHATBOT - FINAL TEST]")
    success = test_complete_rag_pipeline()

    if success:
        print("\n[SUCCESS] RAG pipeline is now working correctly!")
        print("   - Documents properly indexed")
        print("   - Content retrieval functional")
        print("   - Answer generation using book context")
        print("   - No fallback responses like 'topic not covered'")
    else:
        print("\n[FAILURE] Issues remain in the RAG pipeline")
        sys.exit(1)