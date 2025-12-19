import sys
import os
# Add the current directory to Python path to enable imports
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import logging
from typing import List, Dict, Any, Optional
import os
from openai import OpenAI
from models.response import SourceReference
import asyncio

logger = logging.getLogger(__name__)

class AnswerGenerationService:
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-3.5-turbo"):
        """
        Initialize the answer generation service.
        
        Args:
            api_key: OpenAI API key (if None, will use OPENAI_API_KEY environment variable)
            model: OpenAI model to use for generation
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            logger.warning("No OpenAI API key provided. Falling back to rule-based answer generation.")
            self.client = None
        else:
            self.client = OpenAI(api_key=self.api_key)
        
        self.model = model

    def generate_answer(self, query: str, context_chunks: List[Dict[str, Any]], 
                       max_tokens: int = 500) -> Dict[str, Any]:
        """
        Generate an answer based on the query and context chunks.
        
        Args:
            query: User's question
            context_chunks: List of context chunks with content and metadata
            max_tokens: Maximum number of tokens for the response
        
        Returns:
            Dictionary containing the answer, source references, and confidence
        """
        if self.client is not None:
            # Use OpenAI API for answer generation
            return self._generate_with_openai(query, context_chunks, max_tokens)
        else:
            # Use rule-based fallback
            return self._generate_rule_based(query, context_chunks)

    def _generate_with_openai(self, query: str, context_chunks: List[Dict[str, Any]], 
                             max_tokens: int = 500) -> Dict[str, Any]:
        """Generate answer using OpenAI API."""
        try:
            # Format context for the prompt
            context_text = "\n\n".join([
                f"Source: {chunk['source_file']} (Title: {chunk['title']})\nContent: {chunk['content'][:500]}..."
                for chunk in context_chunks
            ])
            
            prompt = f"""
You are an assistant for a book on Physical AI & Humanoid Robotics. Answer the user's question based only on the provided context from the book. If you cannot answer the question based on the context, say "I couldn't find relevant information in the book to answer this question."

Context:
{context_text}

Question: {query}

Answer (be concise and cite the specific source files when possible):
            """.strip()
            
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for a book on Physical AI & Humanoid Robotics. Answer questions based only on the provided context from the book. Be concise and cite specific sources when possible."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=max_tokens,
                temperature=0.3,
            )
            
            answer = response.choices[0].message.content.strip()
            
            # Create source references
            source_references = []
            for chunk in context_chunks[:3]:  # Limit to top 3 sources
                source_ref = SourceReference(
                    chunk_id=chunk.get("doc_id"),
                    text_preview=chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    relevance_score=chunk.get("score", 0.8),
                    location={
                        "source_file": chunk.get("source_file", ""),
                        "title": chunk.get("title", "")
                    }
                )
                source_references.append(source_ref)
            
            return {
                "answer": answer,
                "source_references": source_references,
                "confidence": 0.8  # Placeholder confidence
            }
            
        except Exception as e:
            logger.error(f"Error generating answer with OpenAI: {e}")
            # Fallback to rule-based generation
            return self._generate_rule_based(query, context_chunks)

    def _generate_rule_based(self, query: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Generate answer using rule-based approach when OpenAI is not available."""
        # Simple approach: find chunks that contain query keywords and return relevant text
        query_lower = query.lower()
        relevant_chunks = []
        
        for chunk in context_chunks:
            content_lower = chunk["content"].lower()
            title_lower = chunk["title"].lower()
            
            # Count keyword matches
            keyword_matches = sum(1 for word in query_lower.split() if word in content_lower or word in title_lower)
            
            if keyword_matches > 0 or query_lower in content_lower or query_lower in title_lower:
                relevant_chunks.append((chunk, keyword_matches))
        
        # Sort by keyword matches
        relevant_chunks.sort(key=lambda x: x[1], reverse=True)
        
        if relevant_chunks:
            best_chunk = relevant_chunks[0][0]
            answer = f"Based on the book content: {best_chunk['content'][:300]}..."
            
            # Create source references
            source_references = []
            for chunk, _ in relevant_chunks[:3]:  # Top 3 relevant chunks
                source_ref = SourceReference(
                    chunk_id=chunk.get("doc_id"),
                    text_preview=chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    relevance_score=0.8,  # Fallback relevance
                    location={
                        "source_file": chunk.get("source_file", ""),
                        "title": chunk.get("title", "")
                    }
                )
                source_references.append(source_ref)
            
            return {
                "answer": answer,
                "source_references": source_references,
                "confidence": 0.6  # Lower confidence for rule-based
            }
        else:
            # No relevant content found
            return {
                "answer": "I couldn't find relevant information in the book to answer this question.",
                "source_references": [],
                "confidence": 0.1
            }

    async def generate_answer_async(self, query: str, context_chunks: List[Dict[str, Any]], 
                                   max_tokens: int = 500) -> Dict[str, Any]:
        """Async wrapper for answer generation."""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.generate_answer, query, context_chunks, max_tokens)