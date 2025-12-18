"""
Test for context validation with queries requiring external knowledge.
This tests that the system properly handles queries that cannot be answered from the provided context.
"""
from backend.services.context_validator import ContextValidator
from backend.services.answer_generator import AnswerGenerator
from backend.services.embedding_service import EmbeddingService


def test_context_validation_with_external_knowledge():
    """
    Test context validation with queries that require external knowledge.
    """
    print("Starting context validation tests with external knowledge queries...")

    # Initialize services
    embedding_service = EmbeddingService()
    answer_generator = AnswerGenerator(embedding_service)
    context_validator = ContextValidator()

    # Test 1: Query that requires external knowledge with limited context
    print("\nTest 1: Query requiring external knowledge")
    limited_context = [
        {
            "content": "Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.",
            "text_preview": "AI definition...",
            "relevance_score": 0.8,
            "location": {"chapter": "1", "section": "1.1"}
        }
    ]
    external_query = "What is the current population of New York City?"

    # Generate an answer (ideally this should fail gracefully)
    print(f"Query: '{external_query}'")
    print("Context: Information about AI definition only")

    # Validate the response context
    validation_result = context_validator.validate_response_context(
        answer="The current population of New York City is 8,336,817 (as of 2020 census).",
        context_chunks=limited_context,
        query=external_query
    )

    print(f"Context validation passed: {validation_result['is_context_valid']}")
    print(f"Confidence score: {validation_result['confidence_score']:.3f}")
    print(f"Issues found: {len(validation_result['issues'])}")
    for issue in validation_result["issues"]:
        print(f"  - {issue}")

    # Test 2: Enhanced validation for external knowledge detection
    print("\nTest 2: Enhanced validation for external knowledge")
    enhanced_validation = context_validator.enhanced_validation_check(
        answer="The current population of New York City is 8,336,817 (as of 2020 census).",
        context_chunks=limited_context,
        query=external_query
    )

    print(f"Enhanced validation passed: {enhanced_validation['is_context_valid']}")
    print(f"Enhanced confidence score: {enhanced_validation['confidence_score']:.3f}")
    print(f"Total issues found: {len(enhanced_validation['issues'])}")
    for issue in enhanced_validation["issues"]:
        print(f"  - {issue}")

    # Test 3: Proper handling of insufficient context
    print("\nTest 3: Proper handling of insufficient context")
    safe_failure_result = context_validator.ensure_safe_failure(
        insufficient_context=True,
        query=external_query
    )

    print(f"Safe failure triggered: {safe_failure_result is not None}")
    if safe_failure_result:
        print(f"Safe answer: {safe_failure_result['answer'][:100]}...")
        print(f"Safe confidence: {safe_failure_result['confidence']}")

    # Test 4: Query that can be answered with available context
    print("\nTest 4: Query that can be answered with available context")
    context_query = "What is artificial intelligence?"
    context_for_ai = [
        {
            "content": "Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.",
            "text_preview": "AI definition and capabilities...",
            "relevance_score": 0.9,
            "location": {"chapter": "1", "section": "1.1"}
        }
    ]

    # Simulate a valid answer
    valid_answer = "Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."

    validation_result_valid = context_validator.validate_response_context(
        answer=valid_answer,
        context_chunks=context_for_ai,
        query=context_query
    )

    print(f"Valid context query validation passed: {validation_result_valid['is_context_valid']}")
    print(f"Valid context confidence score: {validation_result_valid['confidence_score']:.3f}")

    # Test 5: Hallucination detection
    print("\nTest 5: Hallucination detection")
    hallucinated_answer = "Artificial Intelligence was first developed in 1956 at the Dartmouth Conference. The current population of New York City is 8,336,817 (as of 2020 census)."

    hallucination_check = context_validator._check_for_hallucination(hallucinated_answer, context_for_ai)
    print(f"Hallucination check passed: {hallucination_check['no_hallucination']}")
    print(f"Hallucination confidence: {hallucination_check['confidence']}")
    print(f"Hallucination issues: {hallucination_check['issues']}")

    print("\nContext validation tests completed!")


if __name__ == "__main__":
    test_context_validation_with_external_knowledge()