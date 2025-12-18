from typing import Any, Dict, List, Optional
import uuid


def validate_session_id(session_id: str) -> bool:
    """
    Validate that the session ID is a properly formatted UUID.

    Args:
        session_id: The session ID to validate

    Returns:
        True if valid, False otherwise
    """
    try:
        uuid.UUID(session_id)
        return True
    except (ValueError, TypeError):
        return False


def validate_selected_text(selected_text: Optional[str]) -> bool:
    """
    Validate that selected text is not too long if provided.

    Args:
        selected_text: The selected text to validate

    Returns:
        True if valid or None, False if too long
    """
    if selected_text is None:
        return True

    # Check length constraint (less than 10000 characters)
    return len(selected_text) < 10000


def validate_query(query: str) -> bool:
    """
    Validate that the query is not empty.

    Args:
        query: The query to validate

    Returns:
        True if valid, False otherwise
    """
    return bool(query and query.strip())


def validate_answer(answer: str) -> bool:
    """
    Validate that the answer is not empty.

    Args:
        answer: The answer to validate

    Returns:
        True if valid, False otherwise
    """
    return bool(answer and answer.strip())


def validate_confidence_score(confidence_score: Optional[float]) -> bool:
    """
    Validate that confidence score is between 0 and 1 if provided.

    Args:
        confidence_score: The confidence score to validate

    Returns:
        True if valid or None, False otherwise
    """
    if confidence_score is None:
        return True

    return 0.0 <= confidence_score <= 1.0


def validate_mode(mode: str) -> bool:
    """
    Validate that mode is one of the allowed values.

    Args:
        mode: The mode to validate

    Returns:
        True if valid, False otherwise
    """
    return mode in ["book-wide", "selected-text"]


def validate_source_references(sources: List[Dict[str, Any]]) -> bool:
    """
    Validate source references for a response.

    Args:
        sources: List of source references to validate

    Returns:
        True if valid, False otherwise
    """
    if not isinstance(sources, list):
        return False

    for source in sources:
        if not isinstance(source, dict):
            return False

        # For book-wide mode, expect chunk_id, for selected-text mode expect different format
        # We'll accept both formats for flexibility
        if "chunk_id" not in source and "source_type" not in source:
            return False

    return True


def validate_book_metadata(metadata: Dict[str, Any]) -> bool:
    """
    Validate book metadata contains required fields.

    Args:
        metadata: The book metadata to validate

    Returns:
        True if valid, False otherwise
    """
    required_fields = ["book_id", "title"]

    for field in required_fields:
        if field not in metadata or not metadata[field]:
            return False

    # Validate total_chunks is non-negative if present
    if "total_chunks" in metadata and metadata["total_chunks"] < 0:
        return False

    return True