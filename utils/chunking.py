import hashlib
from typing import List, Dict, Any
import re


def chunk_text_by_semantic_boundaries(text: str, max_chunk_size: int = 512) -> List[Dict[str, Any]]:
    """
    Chunk text by semantic boundaries while trying to maintain context.

    Args:
        text: The input text to chunk
        max_chunk_size: Maximum size of each chunk in tokens/words

    Returns:
        List of chunks with metadata
    """
    # Split text into sentences
    sentences = re.split(r'[.!?]+\s+', text)

    chunks = []
    current_chunk = ""
    current_metadata = {"start_pos": 0, "end_pos": 0}

    for i, sentence in enumerate(sentences):
        # Check if adding this sentence would exceed the limit
        if len(current_chunk) + len(sentence) < max_chunk_size:
            current_chunk += sentence + ". "
        else:
            # If the current chunk is not empty, save it
            if current_chunk.strip():
                chunk_hash = hashlib.md5(current_chunk.encode()).hexdigest()
                chunks.append({
                    "content": current_chunk.strip(),
                    "metadata": {
                        "hash": chunk_hash,
                        "chunk_index": len(chunks),
                        "sentence_count": len(current_chunk.split('. ')),
                        "word_count": len(current_chunk.split())
                    }
                })

            # Start a new chunk with the current sentence
            current_chunk = sentence + ". "

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunk_hash = hashlib.md5(current_chunk.encode()).hexdigest()
        chunks.append({
            "content": current_chunk.strip(),
            "metadata": {
                "hash": chunk_hash,
                "chunk_index": len(chunks),
                "sentence_count": len(current_chunk.split('. ')),
                "word_count": len(current_chunk.split())
            }
        })

    return chunks


def chunk_text_by_fixed_size(text: str, chunk_size: int = 256) -> List[Dict[str, Any]]:
    """
    Chunk text into fixed-size chunks.

    Args:
        text: The input text to chunk
        chunk_size: Size of each chunk in characters

    Returns:
        List of chunks with metadata
    """
    chunks = []

    for i in range(0, len(text), chunk_size):
        chunk_content = text[i:i + chunk_size]
        chunk_hash = hashlib.md5(chunk_content.encode()).hexdigest()

        chunks.append({
            "content": chunk_content,
            "metadata": {
                "hash": chunk_hash,
                "chunk_index": len(chunks),
                "start_pos": i,
                "end_pos": min(i + chunk_size, len(text))
            }
        })

    return chunks


def validate_chunk_metadata(chunk: Dict[str, Any]) -> bool:
    """
    Validate that a chunk has the required metadata fields.

    Args:
        chunk: The chunk to validate

    Returns:
        True if valid, False otherwise
    """
    required_fields = ["content", "metadata"]
    metadata_required_fields = ["hash"]

    for field in required_fields:
        if field not in chunk:
            return False

    if not isinstance(chunk["content"], str) or not chunk["content"].strip():
        return False

    if not isinstance(chunk["metadata"], dict):
        return False

    for field in metadata_required_fields:
        if field not in chunk["metadata"]:
            return False

    return True