"""
Unit tests for the website embedding pipeline
"""
import pytest
from website_pipeline.chunker import chunk_text


def test_chunk_text():
    """Test that text is properly chunked"""
    text = "This is a sentence. This is another sentence. " * 100  # Create a long text
    chunks = chunk_text(text, max_tokens=50)
    
    # Verify that we have multiple chunks
    assert len(chunks) > 1
    
    # Verify that no chunk exceeds the token limit (approximately)
    for chunk in chunks:
        assert len(chunk.split()) <= 50 + 10  # Allow some buffer for sentence boundaries