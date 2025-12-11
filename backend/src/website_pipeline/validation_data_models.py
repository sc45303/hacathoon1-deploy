"""
Data models for the retrieval pipeline validation system.

This module defines the core data structures used throughout the validation process,
including validation queries, retrieved chunks, validation results, and reports.
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from datetime import datetime


@dataclass
class ValidationQuery:
    """
    Represents a validation query to be tested against the retrieval pipeline.
    
    Attributes:
        id: Unique identifier for the validation query
        text: The query text to be validated
        type: Type of query (factual, conceptual, section-specific)
        expected_section: The expected section or content that should be retrieved
        metadata: Additional metadata about the query
    """
    id: str
    text: str
    type: str  # 'factual', 'conceptual', 'section-specific'
    expected_section: str
    metadata: Dict[str, Any]


@dataclass
class RetrievedChunk:
    """
    Represents a chunk of text retrieved from the vector database.
    
    Attributes:
        id: Unique identifier for the chunk
        content: The text content of the retrieved chunk
        url: The source URL of the content
        position: The position of the chunk in the original document
        relevance_score: Similarity score from the retrieval process
        source_metadata: Metadata from when the chunk was originally stored
    """
    id: str
    content: str
    url: str
    position: int
    relevance_score: float
    source_metadata: Dict[str, Any]


@dataclass
class ValidationResult:
    """
    Represents the results of validating a single query against expected content.
    
    Attributes:
        query_id: Reference to the validation query
        retrieved_chunks: The top retrieved chunks for the query (3-5 items)
        accuracy_score: Overall accuracy score for this query
        correctness: Whether the results are considered accurate
        comparison_details: Detailed comparison between expected and retrieved content
        failure_reason: Reason for failure if results were inaccurate
    """
    query_id: str
    retrieved_chunks: List[RetrievedChunk]
    accuracy_score: float
    correctness: bool
    comparison_details: Dict[str, Any]
    failure_reason: Optional[str] = None


@dataclass
class ValidationReport:
    """
    Represents a comprehensive report of the validation process.
    
    Attributes:
        id: Unique identifier for the report
        date: When the validation was run
        total_queries: Total number of queries tested
        successful_queries: Number of successful queries
        accuracy_percentage: Overall accuracy percentage
        detailed_results: Individual results for each query
        failure_analysis: Summary of common failure patterns
        query_type_breakdown: Accuracy by query type
    """
    id: str
    date: datetime
    total_queries: int
    successful_queries: int
    accuracy_percentage: float
    detailed_results: List[ValidationResult]
    failure_analysis: Dict[str, Any]
    query_type_breakdown: Dict[str, Dict[str, Any]]