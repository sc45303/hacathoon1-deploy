"""
Embedding utility functions for converting text to embeddings using OpenAI models.

This module provides functions to convert text to embeddings using the same OpenAI
embedding model that was used in the ingestion process to maintain consistency.
"""
import os
from typing import List, Union
import openai
from openai import AsyncOpenAI
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Initialize the OpenAI client
client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Get the embedding model from environment variables, default to the one used in ingestion
EMBEDDING_MODEL = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-ada-002")


async def get_embeddings(texts: Union[str, List[str]]) -> List[List[float]]:
    """
    Convert text(s) to embeddings using the OpenAI embedding model.
    
    Args:
        texts: A single text string or a list of text strings to convert to embeddings
        
    Returns:
        A list of embeddings (list of floats) for each input text
    """
    # Ensure texts is a list for consistent processing
    if isinstance(texts, str):
        texts = [texts]
    
    # Create embeddings using OpenAI API
    response = await client.embeddings.create(
        input=texts,
        model=EMBEDDING_MODEL
    )
    
    # Extract and return the embeddings
    embeddings = [data.embedding for data in response.data]
    
    return embeddings


async def get_embedding(text: str) -> List[float]:
    """
    Convert a single text to an embedding using the OpenAI embedding model.
    
    Args:
        text: A single text string to convert to an embedding
        
    Returns:
        An embedding (list of floats) for the input text
    """
    embeddings = await get_embeddings([text])
    return embeddings[0]