from typing import List, Dict, Any
from backend.services.embedding_service import EmbeddingService
import cohere
from backend.config.settings import settings
from backend.utils.validation import validate_query, validate_answer


class AnswerGenerator:
    def __init__(self, embedding_service: EmbeddingService):
        self.embedding_service = embedding_service
        self.client = cohere.Client(settings.cohere_api_key)
        self.model = "command-r-plus"  # Using Cohere's advanced model for question answering

    def generate_answer(self, query: str, context_chunks: List[Dict[str, Any]], max_tokens: int = 500) -> Dict[str, Any]:
        """
        Generate an answer based on the query and provided context chunks.

        Args:
            query: The user's query
            context_chunks: List of relevant context chunks
            max_tokens: Maximum number of tokens for the response

        Returns:
            Dictionary containing the answer and confidence score
        """
        if not validate_query(query):
            raise ValueError("Invalid query provided")

        # Build context from chunks
        context_parts = []
        source_references = []
        for chunk in context_chunks:
            context_parts.append(chunk.get("content", ""))
            source_references.append({
                "chunk_id": chunk.get("chunk_id"),
                "text_preview": chunk.get("text_preview", "")[:100],
                "relevance_score": chunk.get("relevance_score"),
                "location": chunk.get("location", {})
            })

        # Combine all context parts
        context = "\n\n".join(context_parts)

        # Log debug information
        logger.info(f"Generating answer for query: '{query}'")
        logger.info(f"Number of context chunks provided: {len(context_chunks)}")
        if context_chunks:
            logger.info(f"Total context length: {len(context)} characters")
            logger.info(f"Sample context preview: '{context[:200]}...'")

        # Create a prompt that instructs the model to only use provided context
        prompt = f"""
        You are a helpful assistant that answers questions based only on the provided context.
        Do not use any external knowledge or make up information.
        If the answer cannot be found in the provided context, clearly state that.

        Context:
        {context}

        Question: {query}

        Answer:
        """

        try:
            # Generate response using Cohere
            response = self.client.generate(
                model=self.model,
                prompt=prompt,
                max_tokens=max_tokens,
                temperature=0.1,  # Very low temperature for more factual, deterministic responses
                stop_sequences=["Question:", "Context:", "User:", "Assistant:"]
            )

            answer_text = response.generations[0].text.strip()

            # Calculate a basic confidence score based on how much of the context was used
            # This is a simplified approach - in a real system, you might use more sophisticated methods
            confidence_score = self._calculate_confidence(answer_text, context_chunks)

            return {
                "answer": answer_text,
                "source_references": source_references,
                "confidence": confidence_score
            }

        except Exception as e:
            # If there's an error with the LLM, return an error message instead of fallback
            logger.error(f"Error generating answer with LLM: {e}")
            return {
                "answer": "ERROR: LLM service unavailable. RAG pipeline broken.",
                "source_references": [],
                "confidence": 0.0
            }

    def _calculate_confidence(self, answer: str, context_chunks: List[Dict[str, Any]]) -> float:
        """
        Calculate a basic confidence score based on the relevance of context chunks used.

        Args:
            answer: The generated answer
            context_chunks: List of context chunks that were provided

        Returns:
            Confidence score between 0 and 1
        """
        if not context_chunks:
            return 0.0

        # Calculate average relevance score of the chunks
        total_relevance = 0.0
        valid_chunks = 0

        for chunk in context_chunks:
            relevance_score = chunk.get("relevance_score", 0.0)
            if relevance_score > 0:
                total_relevance += relevance_score
                valid_chunks += 1

        if valid_chunks > 0:
            avg_relevance = total_relevance / valid_chunks
            # Normalize the score (assuming relevance scores are between 0 and 1)
            return min(1.0, avg_relevance * 1.2)  # Slightly boost for good matches
        else:
            # If no relevance scores, return a moderate confidence
            return 0.5

    def validate_response_quality(self, query: str, answer: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate the quality of the generated response.

        Args:
            query: Original query
            answer: Generated answer
            context_chunks: Context chunks used to generate the answer

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "is_factual": True,
            "uses_context": True,
            "appropriate_tone": True,
            "complete": True
        }

        # Check if answer is empty
        if not answer.strip():
            validation_results["complete"] = False

        # Check if the answer seems to be based on the context
        answer_lower = answer.lower()
        context_text = " ".join([chunk.get("content", "").lower() for chunk in context_chunks])

        # This is a simplified check - in a real system, you might use more sophisticated NLP
        if len(context_text) > 0:
            # Check if there's some overlap between context and answer
            context_words = set(context_text.split()[:50])  # First 50 words as sample
            answer_words = set(answer_lower.split()[:50])
            overlap = len(context_words.intersection(answer_words))
            validation_results["uses_context"] = overlap > 2  # At least 2 words in common

        return validation_results