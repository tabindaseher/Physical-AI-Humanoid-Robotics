"""
Sample test for the RAG chatbot backend functionality.
This is a basic test to verify the book-wide Q&A functionality works with sample content.
"""
import asyncio
from backend.services.content_indexing_service import ContentIndexingService
from backend.services.embedding_service import EmbeddingService
from backend.services.retrieval_agent import RetrievalAgent
from backend.utils.chunking import chunk_text_by_semantic_boundaries
from sqlmodel import create_engine, Session
from backend.config.database import get_session
from backend.config.settings import settings
import uuid


def test_book_wide_qa_with_sample_content():
    """
    Test the book-wide Q&A functionality with sample book content.
    """
    print("Starting sample content test...")

    # Initialize services
    embedding_service = EmbeddingService()
    retrieval_agent = RetrievalAgent(embedding_service)
    content_indexing_service = ContentIndexingService(embedding_service, retrieval_agent)

    # Sample book content
    sample_book_content = """
    Chapter 1: Introduction to AI
    Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.
    This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.

    There are different types of AI including narrow AI, which is designed for specific tasks like facial recognition or internet searches,
    and general AI, which would have the ability to understand and learn any intellectual task that a human being can.

    Machine learning is a subset of AI that focuses on the use of data and algorithms to imitate the way humans learn, gradually improving accuracy.

    Chapter 2: Machine Learning Basics
    Machine learning algorithms build a model based on training data in order to make predictions or decisions without being explicitly programmed to do so.

    Supervised learning uses labeled datasets to train algorithms to classify data or predict outcomes accurately.
    Unsupervised learning uses machine learning algorithms to analyze and cluster unlabeled datasets.
    Reinforcement learning trains machines through trial and error to take actions that maximize reward.

    Deep learning is a subset of machine learning that uses neural networks with three or more layers. These neural networks attempt to simulate the behavior of the human brain.
    """

    # Index the sample content
    book_id = f"test_book_{uuid.uuid4()}"
    print(f"Indexing sample book with ID: {book_id}")

    indexing_result = content_indexing_service.index_book_content(
        book_id=book_id,
        title="Sample AI Book",
        content=sample_book_content,
        author="Test Author"
    )

    print(f"Indexing completed: {indexing_result}")

    # Test a query
    test_query = "What is artificial intelligence?"

    print(f"Testing query: '{test_query}'")

    # Retrieve similar chunks
    similar_chunks = retrieval_agent.retrieve_similar_chunks(
        query=test_query,
        book_id=book_id,
        limit=3,
        threshold=0.5
    )

    print(f"Found {len(similar_chunks)} similar chunks")
    for i, chunk in enumerate(similar_chunks):
        print(f"Chunk {i+1}: Score={chunk['relevance_score']:.3f}, Preview='{chunk['text_preview']}'")

    print("Sample content test completed successfully!")


if __name__ == "__main__":
    test_book_wide_qa_with_sample_content()