"""
Embedder Module

This module handles the generation of embeddings from text segments.
"""
import os
import cohere
from dotenv import load_dotenv
from typing import List


# Load environment variables and initialize Cohere client
load_dotenv()
co = cohere.Client(os.getenv("COHERE_API_KEY"))


def embed(text_chunks: List[str]) -> List[List[float]]:
    """
    Generate vector representations for text segments using Cohere
    
    Args:
        text_chunks: List of text segments to embed
        
    Returns:
        List of embeddings (each embedding is a list of floats)
    """
    if not text_chunks:
        return []
    
    # Cohere's embed API can handle multiple texts in one request
    response = co.embed(
        texts=text_chunks,
        model='embed-english-v3.0',  # Using Cohere's latest embedding model
        input_type="search_document"  # Appropriate input type for document search
    )
    
    return response.embeddings