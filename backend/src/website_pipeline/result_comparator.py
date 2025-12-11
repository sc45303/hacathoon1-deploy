"""
Content comparison logic for validation.

This module handles comparing retrieved content with original book content
to verify accuracy and relevance of the retrieval results.
"""
import difflib
from typing import List, Dict, Any, Tuple
from .validation_data_models import RetrievedChunk, ValidationQuery
import logging


logger = logging.getLogger(__name__)


def compare_content(retrieved_content: str, expected_content: str) -> Tuple[float, Dict[str, Any]]:
    """
    Compare retrieved content with expected content to determine similarity.
    
    Args:
        retrieved_content: Content retrieved from the system
        expected_content: Expected content to match against
        
    Returns:
        Tuple of (similarity_score, comparison_details)
    """
    # Calculate similarity using difflib
    similarity = difflib.SequenceMatcher(None, retrieved_content.lower(), expected_content.lower()).ratio()
    
    # Additional comparison details
    comparison_details = {
        "text_similarity": similarity,
        "retrieved_length": len(retrieved_content),
        "expected_length": len(expected_content),
        "is_substring_match": expected_content.lower() in retrieved_content.lower() or 
                             retrieved_content.lower() in expected_content.lower()
    }
    
    return similarity, comparison_details


def validate_retrieved_chunks(
    retrieved_chunks: List[RetrievedChunk],
    expected_section: str,
    min_chunk_count: int = 3,
    max_chunk_count: int = 5
) -> Tuple[bool, float, str, Dict[str, Any]]:
    """
    Validate that retrieved chunks meet requirements and match expected content.
    T034: Handles queries with no relevant matches in book content.

    Args:
        retrieved_chunks: List of chunks retrieved from the system
        expected_section: The expected content section
        min_chunk_count: Minimum number of chunks to expect (default: 3)
        max_chunk_count: Maximum number of chunks to expect (default: 5)

    Returns:
        Tuple of (correctness, accuracy_score, failure_reason, comparison_details)
    """
    # Check if the right number of chunks were returned (T013)
    if not (min_chunk_count <= len(retrieved_chunks) <= max_chunk_count):
        # T034: Handle case where no relevant matches exist (empty results)
        if len(retrieved_chunks) == 0:
            return False, 0.0, "No relevant chunks retrieved for the query", {}
        return False, 0.0, f"Incorrect chunk count: expected {min_chunk_count}-{max_chunk_count}, got {len(retrieved_chunks)}", {}

    # Check content accuracy for each chunk
    total_similarity = 0.0
    best_similarity = 0.0
    all_comparison_details = []

    for chunk in retrieved_chunks:
        similarity, comparison_detail = compare_content(chunk.content, expected_section)
        total_similarity += similarity
        best_similarity = max(best_similarity, similarity)
        all_comparison_details.append({
            "chunk_id": chunk.id,
            "similarity": similarity,
            "comparison_detail": comparison_detail
        })

    # Calculate average similarity
    avg_similarity = total_similarity / len(retrieved_chunks) if retrieved_chunks else 0.0

    # Determine correctness based on average similarity
    # Consider it correct if average similarity is above a threshold or if there's a strong individual match
    correctness = avg_similarity > 0.3 or best_similarity > 0.5

    # Compile comparison details
    comparison_details = {
        "average_similarity": avg_similarity,
        "best_similarity": best_similarity,
        "total_chunks": len(retrieved_chunks),
        "chunk_details": all_comparison_details,
        "expected_section_preview": expected_section[:100] + "..." if len(expected_section) > 100 else expected_section
    }

    failure_reason = None if correctness else f"Content similarity below threshold (avg: {avg_similarity:.2f}, best: {best_similarity:.2f})"

    return correctness, avg_similarity, failure_reason, comparison_details


def semantic_similarity_check(embedding1: List[float], embedding2: List[float]) -> float:
    """
    Calculate semantic similarity between two embeddings using cosine similarity.
    
    Args:
        embedding1: First embedding vector
        embedding2: Second embedding vector
        
    Returns:
        Cosine similarity score between 0 and 1
    """
    # Calculate cosine similarity
    dot_product = sum(a * b for a, b in zip(embedding1, embedding2))
    magnitude1 = sum(a * a for a in embedding1) ** 0.5
    magnitude2 = sum(b * b for b in embedding2) ** 0.5
    
    if magnitude1 == 0 or magnitude2 == 0:
        return 0.0
    
    return dot_product / (magnitude1 * magnitude2)