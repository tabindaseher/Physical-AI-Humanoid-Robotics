import os
import logging
from typing import List, Optional
import numpy as np
import asyncio
import aiohttp
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)

class EmbeddingService:
    def __init__(self, model_name: str = "all-MiniLM-L6-v2", expected_dimension: int = 384):
        """
        Initialize the embedding service.
        On Windows, we may need to handle torch DLL issues, so we offer fallback options.

        Args:
            model_name: Name of the sentence transformer model to use
            expected_dimension: Expected dimension of embeddings (to match vector DB)
        """
        self.model_name = model_name
        self.expected_dimension = expected_dimension  # 384 for all-MiniLM-L6-v2, 512 for Cohere models
        self.model = None
        self.device = "cpu"  # Default to CPU to avoid CUDA issues on Windows

        try:
            # Try to load sentence transformers
            from sentence_transformers import SentenceTransformer
            self.model = SentenceTransformer(model_name, device=self.device)
            logger.info(f"Loaded embedding model '{model_name}' on {self.device}")
        except Exception as e:
            logger.warning(f"Failed to load sentence transformer model: {e}")
            logger.info("Embedding service will use OpenAI API if available, or return basic embeddings")
            self.model = None

    def encode_texts(self, texts: List[str], batch_size: int = 32) -> np.ndarray:
        """
        Encode a list of texts into embeddings.
        If sentence transformers fail, return basic fallback embeddings.

        Args:
            texts: List of texts to encode
            batch_size: Batch size for encoding (ignored for fallback)

        Returns:
            Array of embeddings
        """
        if not texts:
            return np.array([])
        
        if self.model is not None:
            try:
                # Generate embeddings using sentence transformers
                embeddings = self.model.encode(
                    texts,
                    batch_size=batch_size,
                    show_progress_bar=len(texts) > 100,
                    convert_to_numpy=True
                )
                return embeddings
            except Exception as e:
                logger.warning(f"Error encoding texts with sentence transformers: {e}")
        
        # Fallback: Use OpenAI if API key is available
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key:
            try:
                import openai
                client = openai.OpenAI(api_key=openai_api_key)
                
                embeddings_list = []
                for text in texts:
                    # Truncate text if too long (OpenAI has limits)
                    truncated_text = text[:8191]  # OpenAI max is 8191 tokens
                    response = client.embeddings.create(
                        input=truncated_text,
                        model="text-embedding-ada-002"
                    )
                    embeddings_list.append(response.data[0].embedding)
                
                return np.array(embeddings_list)
            except Exception as e:
                logger.warning(f"Error encoding texts with OpenAI: {e}")
        
        # Ultimate fallback: return basic numeric representations
        logger.warning("Using fallback embedding method - results will be of lower quality")
        embeddings_list = []
        for text in texts:
            # Simple fallback: convert text to a basic embedding based on character values
            if len(text) == 0:
                embedding = [0.0] * self.expected_dimension
            else:
                # Create a simple hash-based embedding
                text_bytes = text.encode('utf-8')
                embedding = []
                for i in range(self.expected_dimension):
                    # Generate "random" values based on text content
                    byte_idx = i % len(text_bytes)
                    val = (hash(text_bytes[byte_idx:byte_idx+1]) % 20000) / 10000.0 - 1.0
                    embedding.append(val)
            embeddings_list.append(embedding)

        return np.array(embeddings_list)

    def encode_single_text(self, text: str) -> List[float]:
        """
        Encode a single text into an embedding.

        Args:
            text: Single text to encode

        Returns:
            Embedding vector as a list of floats
        """
        embedding = self.encode_texts([text])
        if len(embedding) > 0:
            return embedding[0].tolist()
        else:
            # Return zero vector if encoding failed
            return [0.0] * self.expected_dimension

    async def encode_texts_async(self, texts: List[str], batch_size: int = 32) -> np.ndarray:
        """
        Async wrapper for encoding texts (for compatibility with async code).
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.encode_texts, texts, batch_size)

    def create_embeddings(self, texts: List[str], batch_size: int = 32) -> List[List[float]]:
        """
        Create embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            batch_size: Batch size for encoding

        Returns:
            List of embedding vectors
        """
        embeddings_array = self.encode_texts(texts, batch_size)
        return [embedding.tolist() if hasattr(embedding, 'tolist') else embedding
                for embedding in embeddings_array]

    def create_query_embedding(self, query: str) -> List[float]:
        """
        Create embedding for a single query text.

        Args:
            query: Query text to embed

        Returns:
            Embedding vector as a list of floats
        """
        return self.encode_single_text(query)

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings produced by this model.
        """
        # Default to 384 which is common for all-MiniLM-L6-v2
        return 384