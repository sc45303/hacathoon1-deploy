"""
Embedding generation module for the book ingestion pipeline.

This module provides functions to generate embeddings using OpenAI API.
"""

import os
import logging
import asyncio
import openai
from typing import List, Dict, Any, Union
from tenacity import retry, stop_after_attempt, wait_exponential
from openai import AsyncOpenAI
from app.core.config import settings


logger = logging.getLogger(__name__)

# Initialize OpenAI client with API key from environment
client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)


@retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
async def generate_embedding(text: str) -> List[float]:
    """
    Generates an embedding vector for a text chunk.

    Args:
        text: Text to generate embedding for

    Returns:
        List of floats representing the embedding vector (1536 dimensions)

    Error handling: Raises exception if API call fails, includes retry logic
    """
    try:
        # Use the embedding model specified in environment or default
        model = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-ada-002")
        
        response = await client.embeddings.create(
            input=text,
            model=model
        )
        
        embedding = response.data[0].embedding
        logger.info(f"Generated embedding of size {len(embedding)} for text of length {len(text)}")
        return embedding
    
    except openai.APIError as e:
        logger.error(f"OpenAI API error when generating embedding: {e}")
        raise
    except Exception as e:
        logger.error(f"Error generating embedding: {e}")
        raise


@retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
async def batch_generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generates embeddings for multiple texts in a batch.

    Args:
        texts: List of texts to generate embeddings for

    Returns:
        List of embedding vectors (each a list of floats)

    Error handling: Raises exception if API call fails, includes retry logic
    """
    if not texts:
        return []
    
    try:
        # Use the embedding model specified in environment or default
        model = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-ada-002")
        
        # Note: OpenAI has a limit on batch sizes, typically up to 2048 texts per request
        # For simplicity, we'll handle all texts in one call, but in production
        # you'd want to chunk the requests based on API limits
        response = await client.embeddings.create(
            input=texts,
            model=model
        )
        
        embeddings = [item.embedding for item in response.data]
        logger.info(f"Generated {len(embeddings)} embeddings in batch")
        return embeddings
    
    except openai.APIError as e:
        logger.error(f"OpenAI API error when generating batch embeddings: {e}")
        raise
    except Exception as e:
        logger.error(f"Error generating batch embeddings: {e}")
        raise