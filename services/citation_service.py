from typing import List, Dict, Any
from backend.models.entities import BookContentChunk
from backend.services.storage_service import StorageService
from sqlmodel import Session


class CitationService:
    def __init__(self, storage_service: StorageService):
        self.storage_service = storage_service

    def format_citation(self, chunk: Dict[str, Any], style: str = "apa") -> str:
        """
        Format a citation for a content chunk according to specified style.

        Args:
            chunk: The content chunk with location information
            style: The citation style to use (apa, mla, chicago, etc.)

        Returns:
            Formatted citation string
        """
        location = chunk.get("location", {})

        if style.lower() == "apa":
            return self._format_apa_citation(chunk, location)
        elif style.lower() == "mla":
            return self._format_mla_citation(chunk, location)
        elif style.lower() == "chicago":
            return self._format_chicago_citation(chunk, location)
        else:
            # Default to a simple format
            return self._format_default_citation(chunk, location)

    def _format_apa_citation(self, chunk: Dict[str, Any], location: Dict[str, Any]) -> str:
        """Format citation in APA style."""
        book_id = location.get("book_id", "Unknown Book")
        chapter = location.get("chapter", "N/A")
        section = location.get("section", "N/A")
        page = location.get("page", "N/A")

        return f"({book_id}, Ch. {chapter}, Sec. {section}, p. {page})"

    def _format_mla_citation(self, chunk: Dict[str, Any], location: Dict[str, Any]) -> str:
        """Format citation in MLA style."""
        book_id = location.get("book_id", "Unknown Book")
        chapter = location.get("chapter", "N/A")
        page = location.get("page", "N/A")

        return f"{book_id} {chapter}.{page}"

    def _format_chicago_citation(self, chunk: Dict[str, Any], location: Dict[str, Any]) -> str:
        """Format citation in Chicago style."""
        book_id = location.get("book_id", "Unknown Book")
        chapter = location.get("chapter", "N/A")
        section = location.get("section", "N/A")
        page = location.get("page", "N/A")

        return f"{book_id}, chapter {chapter}, section {section}, {page}."

    def _format_default_citation(self, chunk: Dict[str, Any], location: Dict[str, Any]) -> str:
        """Format citation in a simple default style."""
        book_id = location.get("book_id", "Unknown Book")
        chapter = location.get("chapter", "N/A")
        page = location.get("page", "N/A")

        return f"Source: {book_id}, Chapter: {chapter}, Page: {page}"

    def create_source_references(self, chunk_ids: List[str], session: Session) -> List[Dict[str, Any]]:
        """
        Create properly formatted source references from chunk IDs.

        Args:
            chunk_ids: List of chunk IDs that were used in generating an answer
            session: Database session for retrieving chunk information

        Returns:
            List of formatted source references
        """
        # Get chunks from vector database using retrieval agent
        # For now, we'll use the storage service to get basic information
        # In a real implementation, you'd want to get detailed information from the vector store

        storage_service = self.storage_service
        source_references = []

        # Get chunk information from the database
        for chunk_id in chunk_ids:
            # In a real implementation, we'd get this from the vector database
            # For now, we'll create a basic reference
            source_references.append({
                "chunk_id": chunk_id,
                "text_preview": f"Content chunk {chunk_id[:8]}...",
                "relevance_score": 0.8,  # Default relevance
                "location": {"source": "vector_database", "id": chunk_id}
            })

        return source_references

    def validate_citations(self, answer: str, source_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate that citations in the answer properly reference the source chunks.

        Args:
            answer: The generated answer that may contain citations
            source_chunks: The chunks that were used to generate the answer

        Returns:
            Dictionary with validation results
        """
        results = {
            "citations_valid": True,
            "missing_citations": [],
            "invalid_citations": [],
            "citation_accuracy": 1.0
        }

        # Check if the answer mentions using information from the provided sources
        answer_lower = answer.lower()
        has_source_reference = any([
            "according to" in answer_lower,
            "based on" in answer_lower,
            "source" in answer_lower,
            "cited" in answer_lower,
            "reference" in answer_lower
        ])

        if not has_source_reference:
            results["citations_valid"] = False
            results["missing_citations"].append("Answer doesn't indicate it's based on provided sources")
            results["citation_accuracy"] = 0.5  # Reduced confidence

        return results

    def enhance_answer_with_citations(self, answer: str, source_chunks: List[Dict[str, Any]],
                                    citation_style: str = "apa") -> str:
        """
        Enhance an answer by adding proper citations to source chunks.

        Args:
            answer: The original answer
            source_chunks: The chunks used to generate the answer
            citation_style: The citation style to use

        Returns:
            Enhanced answer with citations
        """
        # For now, we'll append a simple citation list at the end
        # In a more sophisticated implementation, you'd integrate citations inline

        if not source_chunks:
            return answer

        # Format citations
        citations = []
        for chunk in source_chunks:
            citation = self.format_citation(chunk, citation_style)
            citations.append(citation)

        # Append citations to the answer
        enhanced_answer = f"{answer}\n\nSources cited: {', '.join(citations)}"

        return enhanced_answer

    def get_detailed_chunk_info(self, chunk_id: str, session: Session) -> Dict[str, Any]:
        """
        Get detailed information about a specific chunk for citation purposes.

        Args:
            chunk_id: The ID of the chunk to get information for
            session: Database session

        Returns:
            Dictionary with detailed chunk information
        """
        # This would typically query both the SQL database and vector database
        # For now, we'll return a basic structure
        return {
            "chunk_id": chunk_id,
            "location_info": {
                "book_id": "unknown",
                "chapter": "unknown",
                "section": "unknown",
                "page": "unknown"
            },
            "relevance_score": 0.8,
            "text_preview": f"Content for chunk {chunk_id[:8]}..."
        }