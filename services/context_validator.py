from typing import List, Dict, Any, Optional
from backend.utils.validation import validate_answer
import re


class ContextValidator:
    def __init__(self):
        # Define patterns that might indicate external knowledge usage
        self.external_indicators = [
            r"based on general knowledge",
            r"from my training data",
            r"generally speaking",
            r"typically",
            r"usually",
            r"most sources say",
            r"internet search shows",
            r"commonly known",
            r"widely accepted",
            r"according to experts",
            r"scientific consensus",
            r"most people know",
            r"everyone knows",
            r"common knowledge",
            r"well known",
            r"known to be",
            r"it's known that",
        ]

    def validate_response_context(self, answer: str, context_chunks: List[Dict[str, Any]], query: str) -> Dict[str, Any]:
        """
        Validate that the response is grounded in the provided context.

        Args:
            answer: The generated answer to validate
            context_chunks: The context chunks provided to generate the answer
            query: The original query

        Returns:
            Dictionary with validation results and confidence score
        """
        if not validate_answer(answer):
            return {
                "is_context_valid": False,
                "confidence_score": 0.0,
                "issues": ["Answer is empty or invalid"],
                "suggestions": ["Generate a proper answer based on the context"]
            }

        issues = []
        suggestions = []

        # Check for external knowledge indicators
        external_usage_detected = self._check_external_knowledge_indicators(answer)
        if external_usage_detected:
            issues.append("Response contains phrases indicating external knowledge usage")
            suggestions.append("Rephrase to only reference information in the provided context")

        # Check if the answer is too generic
        is_too_generic = self._check_generality(answer)
        if is_too_generic:
            issues.append("Response is too generic and may not be specifically based on the context")
            suggestions.append("Provide specific details from the context chunks")

        # Check if answer is related to the query and context
        query_context_alignment = self._check_query_context_alignment(query, answer, context_chunks)
        if query_context_alignment < 0.3:  # If alignment is low
            issues.append("Response doesn't seem well-aligned with the query or context")
            suggestions.append("Ensure the answer directly addresses the query using information from the context")

        is_context_valid = len(issues) == 0
        confidence_score = self._calculate_validation_confidence(answer, context_chunks, external_usage_detected, is_too_generic, query_context_alignment)

        return {
            "is_context_valid": is_context_valid,
            "confidence_score": confidence_score,
            "issues": issues,
            "suggestions": suggestions
        }

    def _check_external_knowledge_indicators(self, answer: str) -> bool:
        """
        Check if the answer contains indicators of external knowledge usage.

        Args:
            answer: The answer to check

        Returns:
            True if external knowledge indicators are found, False otherwise
        """
        answer_lower = answer.lower()
        for indicator in self.external_indicators:
            if re.search(indicator, answer_lower):
                return True
        return False

    def _check_generality(self, answer: str) -> bool:
        """
        Check if the answer is too general and not specific to the context.

        Args:
            answer: The answer to check

        Returns:
            True if the answer is too general, False otherwise
        """
        # Convert to lowercase and remove extra whitespace
        answer_clean = " ".join(answer.lower().split())

        # Check for generic phrases that don't reference specific context
        generic_indicators = [
            "it depends",
            "varies based on",
            "there are many",
            "several factors",
            "depends on the situation",
            "it can vary",
            "different approaches",
            "many different",
        ]

        for indicator in generic_indicators:
            if indicator in answer_clean:
                return True

        # Check if answer is too short to contain specific information
        words = answer_clean.split()
        if len(words) < 10:
            # If it's short and doesn't contain specific terms from context, it might be too generic
            return True

        return False

    def _check_query_context_alignment(self, query: str, answer: str, context_chunks: List[Dict[str, Any]]) -> float:
        """
        Check how well the answer aligns with both the query and context.

        Args:
            query: The original query
            answer: The generated answer
            context_chunks: The context chunks provided

        Returns:
            Alignment score between 0 and 1
        """
        query_lower = query.lower()
        answer_lower = answer.lower()

        # Get all content from context chunks
        context_content = " ".join([chunk.get("content", "").lower() for chunk in context_chunks])

        # Count overlapping terms between query and answer
        query_terms = set(query_lower.split())
        answer_terms = set(answer_lower.split())

        # Count overlapping terms between answer and context
        context_terms = set(context_content.split())

        # Calculate alignment scores
        query_answer_overlap = len(query_terms.intersection(answer_terms))
        answer_context_overlap = len(answer_terms.intersection(context_terms))

        # Normalize scores (simple approach)
        query_answer_alignment = min(1.0, query_answer_overlap / max(len(query_terms), 1))
        answer_context_alignment = min(1.0, answer_context_overlap / max(len(answer_terms), 1))

        # Combined alignment score
        alignment_score = (query_answer_alignment * 0.4) + (answer_context_alignment * 0.6)

        return alignment_score

    def _calculate_validation_confidence(self, answer: str, context_chunks: List[Dict[str, Any]],
                                       external_usage_detected: bool, is_too_generic: bool,
                                       query_context_alignment: float) -> float:
        """
        Calculate the overall validation confidence score.

        Args:
            answer: The answer being validated
            context_chunks: The context chunks provided
            external_usage_detected: Whether external knowledge indicators were found
            is_too_generic: Whether the answer is too generic
            query_context_alignment: How well the answer aligns with query and context

        Returns:
            Confidence score between 0 and 1
        """
        base_score = 1.0

        # Penalize for external knowledge usage
        if external_usage_detected:
            base_score -= 0.4

        # Penalize for being too generic
        if is_too_generic:
            base_score -= 0.3

        # Adjust based on alignment
        base_score = base_score * query_context_alignment

        # Ensure score is within bounds
        return max(0.0, min(1.0, base_score))

    def validate_context_isolation(self, mode: str, answer: str, provided_context: str, global_context_available: bool) -> Dict[str, Any]:
        """
        Validate that in selected-text mode, the answer only uses the provided context and not global context.

        Args:
            mode: The current mode ("book-wide" or "selected-text")
            answer: The generated answer
            provided_context: The specific context provided to the model
            global_context_available: Whether global context was available during generation

        Returns:
            Dictionary with validation results
        """
        if mode != "selected-text":
            return {
                "context_isolation_valid": True,
                "confidence": 1.0,
                "issues": [],
                "suggestions": []
            }

        # For selected-text mode, validate that the answer is based on the provided context
        provided_context_lower = provided_context.lower()
        answer_lower = answer.lower()

        # Check if answer contains specific terms from the provided context
        provided_terms = set(provided_context_lower.split()[:50])  # Use first 50 words as sample
        answer_terms = set(answer_lower.split()[:50])

        overlap = len(provided_terms.intersection(answer_terms))
        overlap_ratio = overlap / max(len(provided_terms), 1)

        issues = []
        suggestions = []

        if overlap_ratio < 0.1:  # Less than 10% overlap
            issues.append("Answer appears to not be based on the provided context")
            suggestions.append("Ensure the answer directly references information from the selected text")

        return {
            "context_isolation_valid": overlap_ratio >= 0.1,
            "confidence": overlap_ratio,
            "issues": issues,
            "suggestions": suggestions
        }

    def ensure_safe_failure(self, insufficient_context: bool, query: str) -> Dict[str, Any]:
        """
        Generate a safe response when context is insufficient.

        Args:
            insufficient_context: Whether context is insufficient for a good answer
            query: The original query

        Returns:
            Safe response dictionary
        """
        if insufficient_context:
            return {
                "answer": "I cannot provide an answer based on the available context. The provided information is insufficient to answer your question about: " + query,
                "source_references": [],
                "confidence": 0.0,
                "is_context_valid": False
            }
        else:
            return None  # No need for safe failure

    def enhanced_validation_check(self, answer: str, context_chunks: List[Dict[str, Any]], query: str) -> Dict[str, Any]:
        """
        Enhanced validation that performs multiple checks to ensure answer quality and context adherence.

        Args:
            answer: The generated answer to validate
            context_chunks: The context chunks provided to generate the answer
            query: The original query

        Returns:
            Dictionary with detailed validation results
        """
        # Perform all existing validations
        basic_validation = self.validate_response_context(answer, context_chunks, query)

        # Additional enhanced checks
        factuality_check = self._check_answer_factuality(answer, context_chunks)
        relevance_check = self._check_answer_relevance(answer, query)
        hallucination_check = self._check_for_hallucination(answer, context_chunks)

        # Combine all validation results
        all_issues = basic_validation["issues"]
        all_issues.extend(factuality_check.get("issues", []))
        all_issues.extend(relevance_check.get("issues", []))
        all_issues.extend(hallucination_check.get("issues", []))

        all_suggestions = basic_validation["suggestions"]
        all_suggestions.extend(factuality_check.get("suggestions", []))
        all_suggestions.extend(relevance_check.get("suggestions", []))
        all_suggestions.extend(hallucination_check.get("suggestions", []))

        # Calculate enhanced confidence score
        base_confidence = basic_validation["confidence_score"]
        factuality_score = factuality_check.get("confidence", 1.0)
        relevance_score = relevance_check.get("confidence", 1.0)
        hallucination_score = hallucination_check.get("confidence", 1.0)

        # Combine scores with weights
        enhanced_confidence = (
            base_confidence * 0.4 +
            factuality_score * 0.2 +
            relevance_score * 0.2 +
            hallucination_score * 0.2
        )

        return {
            "is_context_valid": basic_validation["is_context_valid"] and
                               factuality_check.get("is_factual", True) and
                               relevance_check.get("is_relevant", True) and
                               hallucination_check.get("no_hallucination", True),
            "confidence_score": enhanced_confidence,
            "issues": all_issues,
            "suggestions": all_suggestions,
            "detailed_checks": {
                "basic_context_validation": basic_validation,
                "factuality_check": factuality_check,
                "relevance_check": relevance_check,
                "hallucination_check": hallucination_check
            }
        }

    def _check_answer_factuality(self, answer: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Check if the answer contains factual information that can be verified against the context.

        Args:
            answer: The answer to check
            context_chunks: The context chunks that should support the answer

        Returns:
            Dictionary with factuality check results
        """
        issues = []
        suggestions = []

        # Check if specific claims in the answer can be found in the context
        answer_sentences = [s.strip() for s in answer.split('.') if s.strip()]
        context_text = " ".join([chunk.get("content", "") for chunk in context_chunks])
        context_lower = context_text.lower()

        # Look for specific claims that might not be supported by context
        unsupported_claims = []
        for sentence in answer_sentences:
            if len(sentence) > 10:  # Only check meaningful sentences
                sentence_lower = sentence.lower()
                # Check if this sentence has substantial overlap with context
                words = sentence_lower.split()
                if len(words) > 3:  # Only check sentences with multiple words
                    context_words = set(context_lower.split())
                    sentence_words = set(words)
                    overlap = len(context_words.intersection(sentence_words))

                    if overlap < max(2, len(sentence_words) * 0.3):  # Less than 30% word overlap
                        unsupported_claims.append(sentence)

        if unsupported_claims:
            issues.append(f"Answer contains {len(unsupported_claims)} potentially unsupported claims")
            suggestions.append("Verify all claims in the answer against the provided context")

        return {
            "is_factual": len(unsupported_claims) == 0,
            "confidence": 0.0 if unsupported_claims else 1.0,
            "issues": issues,
            "suggestions": suggestions
        }

    def _check_answer_relevance(self, answer: str, query: str) -> Dict[str, Any]:
        """
        Check if the answer is relevant to the original query.

        Args:
            answer: The answer to check
            query: The original query

        Returns:
            Dictionary with relevance check results
        """
        issues = []
        suggestions = []

        answer_lower = answer.lower()
        query_lower = query.lower()

        # Check if answer addresses key terms from the query
        query_words = set(query_lower.split())
        answer_words = set(answer_lower.split())

        # Calculate overlap between query and answer
        overlap = len(query_words.intersection(answer_words))
        query_coverage = overlap / max(len(query_words), 1)

        if query_coverage < 0.1:  # Less than 10% overlap
            issues.append("Answer has low relevance to the query")
            suggestions.append("Ensure the answer directly addresses the query")

        return {
            "is_relevant": query_coverage >= 0.1,
            "confidence": query_coverage,
            "issues": issues,
            "suggestions": suggestions
        }

    def _check_for_hallucination(self, answer: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Check for signs of hallucination in the answer.

        Args:
            answer: The answer to check for hallucination
            context_chunks: The context chunks that should support the answer

        Returns:
            Dictionary with hallucination check results
        """
        issues = []
        suggestions = []

        # Check for common hallucination indicators
        hallucination_indicators = [
            r"according to my training data",
            r"based on general knowledge",
            r"from what i know",
            r"most experts agree",
            r"studies show",
            r"research indicates",
            r"it is commonly believed",
            r"many people think",
            r"internet sources say",
            r"external sources indicate",
        ]

        import re
        answer_lower = answer.lower()

        for indicator in hallucination_indicators:
            if re.search(indicator, answer_lower):
                issues.append(f"Answer contains potential hallucination indicator: '{indicator}'")
                suggestions.append("Rephrase to only reference information in the provided context")

        # Check if answer contains very specific information not in context
        context_text = " ".join([chunk.get("content", "") for chunk in context_chunks])
        context_lower = context_text.lower()

        # Look for named entities that might not be in context
        import re
        # This is a simplified check - in a real system you'd use NLP for named entity recognition
        potential_issues = []

        # Check for dates, percentages, statistics that might be hallucinated
        stats_patterns = re.findall(r'\b\d{4}\b|\b\d+%|\b\d+\.\d+\b', answer)  # Years, percentages, decimals
        for pattern in stats_patterns:
            if pattern.lower() not in context_lower:
                potential_issues.append(f"Specific statistic '{pattern}' not found in context")

        if potential_issues:
            issues.extend(potential_issues)
            suggestions.append("Verify all specific numbers and statistics against the provided context")

        return {
            "no_hallucination": len(issues) == 0,
            "confidence": 0.0 if issues else 1.0,
            "issues": issues,
            "suggestions": suggestions
        }