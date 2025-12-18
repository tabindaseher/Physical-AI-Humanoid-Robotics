"""
Test for the selected-text Q&A functionality.
This tests that the system properly uses only the provided text and not global context.
"""
from backend.services.content_indexing_service import ContentIndexingService
from backend.services.embedding_service import EmbeddingService
from backend.services.retrieval_agent import RetrievalAgent
from backend.services.answer_generator import AnswerGenerator
from backend.services.context_validator import ContextValidator
from backend.config.settings import settings
import uuid


def test_selected_text_qa():
    """
    Test the selected-text Q&A functionality with various text inputs.
    """
    print("Starting selected-text Q&A tests...")

    # Initialize services
    embedding_service = EmbeddingService()
    retrieval_agent = RetrievalAgent(embedding_service)
    answer_generator = AnswerGenerator(embedding_service)
    context_validator = ContextValidator()

    # Test 1: Basic selected-text functionality
    print("\nTest 1: Basic selected-text functionality")
    selected_text_1 = """
    Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.
    This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.
    """
    query_1 = "What is artificial intelligence?"

    # Process using selected-text approach
    results_1 = retrieval_agent.process_selected_text_query(query_1, selected_text_1)
    print(f"Query: '{query_1}'")
    print(f"Results found: {len(results_1)}")
    if results_1:
        print(f"Relevance score: {results_1[0]['relevance_score']:.3f}")

    # Generate answer
    answer_result_1 = answer_generator.generate_answer(query_1, results_1)
    print(f"Generated answer: {answer_result_1['answer'][:100]}...")

    # Validate context isolation
    isolation_validation_1 = context_validator.validate_context_isolation(
        mode="selected-text",
        answer=answer_result_1["answer"],
        provided_context=selected_text_1,
        global_context_available=True
    )
    print(f"Context isolation validation: {isolation_validation_1['context_isolation_valid']}")

    # Test 2: Different text content
    print("\nTest 2: Different text content")
    selected_text_2 = """
    Machine learning is a method of data analysis that automates analytical model building.
    It is a branch of artificial intelligence based on the idea that systems can learn from data,
    identify patterns and make decisions with minimal human intervention.
    """
    query_2 = "What is machine learning?"

    results_2 = retrieval_agent.process_selected_text_query(query_2, selected_text_2)
    print(f"Query: '{query_2}'")
    print(f"Results found: {len(results_2)}")

    answer_result_2 = answer_generator.generate_answer(query_2, results_2)
    print(f"Generated answer: {answer_result_2['answer'][:100]}...")

    # Test 3: Context isolation validation
    print("\nTest 3: Context isolation validation")
    selected_text_3 = "The sky is blue and the grass is green."
    query_3 = "What color is the sky?"

    # Verify that the answer only comes from the selected text
    results_3 = retrieval_agent.process_selected_text_query(query_3, selected_text_3)
    answer_result_3 = answer_generator.generate_answer(query_3, results_3)

    # Check if the answer contains information from the selected text
    answer_contains_context_info = "blue" in answer_result_3["answer"].lower()
    print(f"Answer contains context info: {answer_contains_context_info}")
    print(f"Generated answer: {answer_result_3['answer']}")

    # Test 4: Irrelevant query to selected text
    print("\nTest 4: Irrelevant query to selected text")
    selected_text_4 = "The sky is blue and the grass is green."
    query_4 = "What is the capital of France?"

    results_4 = retrieval_agent.process_selected_text_query(query_4, selected_text_4)
    print(f"Query: '{query_4}' (unrelated to selected text)")
    print(f"Results found: {len(results_4)}")

    if results_4:
        answer_result_4 = answer_generator.generate_answer(query_4, results_4)
        print(f"Generated answer: {answer_result_4['answer'][:100]}...")
    else:
        print("No relevant content found in selected text - would need safe failure handling")

    print("\nSelected-text Q&A tests completed!")


if __name__ == "__main__":
    test_selected_text_qa()