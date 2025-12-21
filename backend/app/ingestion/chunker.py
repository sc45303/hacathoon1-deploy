"""
Text chunking module for the book ingestion pipeline.

This module provides functions to chunk large text into smaller segments
with configurable size and overlap, with proper token counting.

Chunking approach:
This module implements a character-based chunking algorithm that divides text
into segments of approximately equal token lengths. The algorithm creates
overlapping regions between consecutive chunks to preserve context continuity
at chunk boundaries. The default chunk size is 400 tokens with 50-token overlap.
"""

import logging
import hashlib
import uuid
from typing import Dict, List, Any
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class Chunk:
    """Represents a text chunk with metadata."""
    chunk_id: str
    chunk_hash: str
    content: str
    source_file: str
    chunk_index: int
    token_count: int


def calculate_tokens(text: str) -> int:
    """
    Calculates the number of tokens in a text string.

    Args:
        text: Text to count tokens in

    Returns:
        Integer number of tokens

    Error handling: Returns 0 if calculation fails
    """
    try:
        # For this implementation, we'll use a simple heuristic:
        # Average token is about 4 characters for English text
        # This is a simplified approach - in a production system you'd use
        # tiktoken or another tokenizer specifically for your model
        if not text:
            return 0
        # Rough estimation: count words and assume ~1.3 tokens per word
        words = len(text.split())
        # Or count characters divided by 4 (average token length)
        chars_token_estimate = len(text) // 4
        # Use the greater of the two estimates for safety
        return max(words, chars_token_estimate)
    except Exception as e:
        logger.error(f"Error calculating tokens for text: {e}")
        return 0


def chunk_text(text: str, source_file: str, chunk_size: int = 400, overlap: int = 50) -> List[Dict[str, Any]]:
    """
    Splits a text into chunks of specified size with overlap.

    Args:
        text: Text to be chunked
        source_file: Path of the source file (for metadata)
        chunk_size: Target number of tokens per chunk (default: 400)
        overlap: Number of tokens to overlap between chunks (default: 50)

    Returns:
        List of dictionaries containing:
        - chunk_id: Generated UUID for the chunk
        - chunk_hash: SHA-256 hash of content+source for idempotency
        - content: Text content of the chunk
        - source_file: Path of the source file
        - chunk_index: Sequential index of this chunk in the source file
        - token_count: Number of tokens in the chunk

    Error handling: Raises exception if text processing fails
    """
    if not text:
        return []

    try:
        # For this implementation, we'll split based on characters since we're using a simple token estimator
        # In a real implementation, we'd use a proper tokenizer like tiktoken
        text_length = len(text)
        chunks = []
        start_idx = 0
        chunk_index = 0

        while start_idx < text_length:
            # Determine the end index for this chunk
            end_idx = start_idx + chunk_size * 4  # Approximate character count based on our token estimate

            # Adjust for overlap in next iteration
            next_start = end_idx - overlap * 4

            # If we're near the end, make sure to include the remainder
            if end_idx >= text_length:
                end_idx = text_length

            # Extract the chunk content
            chunk_content = text[start_idx:end_idx]

            # Calculate actual token count for this chunk
            token_count = calculate_tokens(chunk_content)

            # Create a hash for idempotency (based on content and source)
            content_hash = hashlib.sha256(f"{chunk_content}{source_file}".encode()).hexdigest()

            # Create the chunk data
            chunk_data = {
                "chunk_id": str(uuid.uuid4()),
                "chunk_hash": content_hash,
                "content": chunk_content,
                "source_file": source_file,
                "chunk_index": chunk_index,
                "token_count": token_count
            }

            chunks.append(chunk_data)

            # Move to the next chunk position
            start_idx = next_start
            chunk_index += 1

            # Safety check to prevent infinite loop
            if start_idx >= text_length:
                break

        logger.info(f"Text chunked into {len(chunks)} chunks from {source_file}")
        return chunks

    except Exception as e:
        logger.error(f"Error chunking text from {source_file}: {e}")
        raise