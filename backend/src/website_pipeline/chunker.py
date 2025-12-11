"""
Chunker Module

This module handles the division of text into appropriately sized segments.
"""
from typing import List


def chunk_text(text: str, max_tokens: int = 512) -> List[str]:
    """
    Divide text into appropriately sized segments
    
    Args:
        text: The text to chunk
        max_tokens: Maximum number of tokens per chunk (approximated as words)
        
    Returns:
        List of text chunks
    """
    # Simple tokenization by sentences to create chunks
    sentences = text.split('. ')
    chunks = []
    current_chunk = []
    current_length = 0
    
    for sentence in sentences:
        # Estimate token count (approximate: 1 token = 1 word)
        sentence_length = len(sentence.split())
        
        # If adding this sentence exceeds the limit, start a new chunk
        if current_length + sentence_length > max_tokens and current_chunk:
            chunks.append('. '.join(current_chunk) + '.')
            current_chunk = [sentence]
            current_length = sentence_length
        else:
            current_chunk.append(sentence)
            current_length += sentence_length
    
    # Add the last chunk if it has content
    if current_chunk:
        chunks.append('. '.join(current_chunk) + '.')
    
    return chunks