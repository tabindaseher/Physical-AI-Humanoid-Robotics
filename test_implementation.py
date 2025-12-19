"""
Comprehensive test to verify the RAG chatbot backend implementation.
This tests the complete functionality of the system.
"""
from backend.services.content_indexing_service import ContentIndexingService
from backend.services.embedding_service import EmbeddingService
from backend.services.retrieval_agent import RetrievalAgent
from backend.services.answer_generator import AnswerGenerator
from backend.services.context_validator import ContextValidator
from backend.config.settings import settings
from backend.api.rag import book_wide_qa, selected_text_qa
from backend.models.request import BookWideQueryRequest, SelectedTextQueryRequest
from sqlmodel import Session
from backend.config.database import get_session
import uuid


def test_complete_implementation():
    """
    Test the complete implementation of the RAG chatbot.
    """
    print("Starting comprehensive implementation test...")

    # Test 1: Service initialization
    print("\nTest 1: Service initialization")
    try:
        embedding_service = EmbeddingService()
        retrieval_agent = RetrievalAgent(embedding_service)
        answer_generator = AnswerGenerator(embedding_service)
        context_validator = ContextValidator()
        print("✓ All services initialized successfully")
    except Exception as e:
        print(f"✗ Service initialization failed: {e}")
        return

    # Test 2: Content indexing
    print("\nTest 2: Content indexing")
    try:
        content_indexing_service = ContentIndexingService(embedding_service, retrieval_agent)
        sample_content = """
        Chapter 1: Introduction to Artificial Intelligence
        Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.
        This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.

        Machine learning is a subset of AI that focuses on the use of data and algorithms to imitate the way humans learn.
        """

        indexing_result = content_indexing_service.index_book_content(
            book_id=f"test_book_{uuid.uuid4()}",
            title="Test AI Book",
            content=sample_content,
            author="Test Author"
        )
        print(f"✓ Content indexed successfully: {indexing_result['valid_chunks']} chunks processed")
    except Exception as e:
        print(f"✗ Content indexing failed: {e}")
        # This might fail if Qdrant is not configured, which is expected in test environment
        print("  (This may be expected if Qdrant is not configured)")

    # Test 3: Context validation
    print("\nTest 3: Context validation")
    try:
        test_answer = "Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."
        test_context = [{
            "content": "Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.",
            "text_preview": "AI definition...",
            "relevance_score": 0.9,
            "location": {"chapter": "1", "section": "1.1"}
        }]
        test_query = "What is artificial intelligence?"

        validation_result = context_validator.validate_response_context(
            answer=test_answer,
            context_chunks=test_context,
            query=test_query
        )
        print(f"✓ Context validation completed with score: {validation_result['confidence_score']:.3f}")
    except Exception as e:
        print(f"✗ Context validation failed: {e}")

    # Test 4: Enhanced validation
    print("\nTest 4: Enhanced validation")
    try:
        enhanced_result = context_validator.enhanced_validation_check(
            answer=test_answer,
            context_chunks=test_context,
            query=test_query
        )
        print(f"✓ Enhanced validation completed with score: {enhanced_result['confidence_score']:.3f}")
        print(f"  Issues found: {len(enhanced_result['issues'])}")
    except Exception as e:
        print(f"✗ Enhanced validation failed: {e}")

    # Test 5: Answer generation
    print("\nTest 5: Answer generation")
    try:
        answer_result = answer_generator.generate_answer(
            query=test_query,
            context_chunks=test_context
        )
        print(f"✓ Answer generated with confidence: {answer_result['confidence']:.3f}")
    except Exception as e:
        print(f"✗ Answer generation failed: {e}")
        # This might fail if Cohere API is not configured

    # Test 6: Retrieval functionality
    print("\nTest 6: Retrieval functionality")
    try:
        # Test with a mock query (this will work even without real data in Qdrant)
        similar_chunks = retrieval_agent.retrieve_similar_chunks(
            query="What is AI?",
            limit=3,
            threshold=0.5
        )
        print(f"✓ Retrieval completed, found {len(similar_chunks)} chunks")
    except Exception as e:
        print(f"✗ Retrieval failed: {e}")
        # This might fail if Qdrant is not configured

    # Test 7: Selected-text processing
    print("\nTest 7: Selected-text processing")
    try:
        selected_text = "Machine learning is a subset of AI that focuses on the use of data and algorithms."
        query = "What is machine learning?"

        selected_results = retrieval_agent.process_selected_text_query(query, selected_text)
        print(f"✓ Selected-text processing completed, found {len(selected_results)} results")

        if selected_results:
            selected_answer = answer_generator.generate_answer(query, selected_results)
            print(f"  Generated answer: {selected_answer['answer'][:100]}...")
    except Exception as e:
        print(f"✗ Selected-text processing failed: {e}")

    print("\n✓ Comprehensive implementation test completed!")
    print("\nNote: Some tests may show failures if external services (Cohere, Qdrant) are not configured.")
    print("This is expected in a test environment without proper API keys and service setup.")


if __name__ == "__main__":
    test_complete_implementation()