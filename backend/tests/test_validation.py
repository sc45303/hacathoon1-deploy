"""
Unit and integration tests for the validation functionality.
"""
import pytest
import os
from unittest.mock import Mock, MagicMock, patch
from backend.src.website_pipeline.validation_data_models import ValidationQuery, RetrievedChunk, ValidationResult, ValidationReport
from backend.src.website_pipeline.retrieval_validator import RetrievalValidator
from backend.src.website_pipeline.result_comparator import validate_retrieved_chunks, compare_content
from backend.src.website_pipeline.report_generator import generate_validation_report, analyze_failures
from backend.src.website_pipeline.validation_runner import load_test_queries


class TestValidationDataModels:
    """Tests for data model classes."""
    
    def test_validation_query_creation(self):
        """Test creating a ValidationQuery object."""
        query = ValidationQuery(
            id="test_001",
            text="What is this about?",
            type="factual",
            expected_section="Introduction section",
            metadata={"difficulty": "easy"}
        )
        
        assert query.id == "test_001"
        assert query.text == "What is this about?"
        assert query.type == "factual"
        assert query.expected_section == "Introduction section"
        assert query.metadata == {"difficulty": "easy"}
    
    def test_retrieved_chunk_creation(self):
        """Test creating a RetrievedChunk object."""
        chunk = RetrievedChunk(
            id="chunk_001",
            content="This is sample content",
            url="https://example.com",
            position=1,
            relevance_score=0.85,
            source_metadata={"author": "test"}
        )
        
        assert chunk.id == "chunk_001"
        assert chunk.content == "This is sample content"
        assert chunk.url == "https://example.com"
        assert chunk.position == 1
        assert chunk.relevance_score == 0.85
        assert chunk.source_metadata == {"author": "test"}
    
    def test_validation_result_creation(self):
        """Test creating a ValidationResult object."""
        chunk = RetrievedChunk(
            id="chunk_001",
            content="This is sample content",
            url="https://example.com",
            position=1,
            relevance_score=0.85,
            source_metadata={}
        )
        
        result = ValidationResult(
            query_id="query_001",
            retrieved_chunks=[chunk],
            accuracy_score=0.75,
            correctness=True,
            comparison_details={"text_similarity": 0.75},
            failure_reason=None
        )
        
        assert result.query_id == "query_001"
        assert len(result.retrieved_chunks) == 1
        assert result.accuracy_score == 0.75
        assert result.correctness is True
        assert result.comparison_details == {"text_similarity": 0.75}
        assert result.failure_reason is None
    
    def test_validation_report_creation(self):
        """Test creating a ValidationReport object."""
        result = ValidationResult(
            query_id="query_001",
            retrieved_chunks=[],
            accuracy_score=0.75,
            correctness=True,
            comparison_details={"text_similarity": 0.75},
            failure_reason=None
        )
        
        report = ValidationReport(
            id="report_001",
            date=MagicMock(),
            total_queries=1,
            successful_queries=1,
            accuracy_percentage=100.0,
            detailed_results=[result],
            failure_analysis={},
            query_type_breakdown={"factual": {"accuracy": 100.0, "count": 1, "correct": 1}}
        )
        
        assert report.id == "report_001"
        assert report.total_queries == 1
        assert report.successful_queries == 1
        assert report.accuracy_percentage == 100.0
        assert len(report.detailed_results) == 1


class TestResultComparator:
    """Tests for the result comparator functionality."""
    
    def test_compare_content(self):
        """Test content comparison function."""
        retrieved = "This is the content we retrieved from the system"
        expected = "This is the content we expected to retrieve"
        
        similarity, details = compare_content(retrieved, expected)
        
        assert 0.0 <= similarity <= 1.0
        assert isinstance(details, dict)
        assert "text_similarity" in details
        assert "retrieved_length" in details
        assert "expected_length" in details
        assert "is_substring_match" in details
    
    def test_validate_retrieved_chunks_correct_count(self):
        """Test validation when correct number of chunks are retrieved."""
        chunk1 = RetrievedChunk(
            id="chunk_001",
            content="Content one",
            url="https://example.com/1",
            position=1,
            relevance_score=0.9,
            source_metadata={}
        )
        
        chunk2 = RetrievedChunk(
            id="chunk_002",
            content="Content two", 
            url="https://example.com/2",
            position=2,
            relevance_score=0.8,
            source_metadata={}
        )
        
        chunks = [chunk1, chunk2, chunk2]  # 3 chunks
        expected_section = "This is the expected section"
        
        correctness, accuracy, failure_reason, details = validate_retrieved_chunks(
            chunks, expected_section, min_chunk_count=3, max_chunk_count=5
        )
        
        assert correctness is True  # Should be correct based on content similarity
        assert 0.0 <= accuracy <= 1.0
        assert failure_reason is None  # No failure if count is correct
        assert isinstance(details, dict)
    
    def test_validate_retrieved_chunks_incorrect_count(self):
        """Test validation when incorrect number of chunks are retrieved."""
        chunk1 = RetrievedChunk(
            id="chunk_001",
            content="Content one",
            url="https://example.com/1",
            position=1,
            relevance_score=0.9,
            source_metadata={}
        )
        
        chunks = [chunk1]  # Only 1 chunk, expected 3-5
        expected_section = "This is the expected section"
        
        correctness, accuracy, failure_reason, details = validate_retrieved_chunks(
            chunks, expected_section, min_chunk_count=3, max_chunk_count=5
        )
        
        assert correctness is False
        assert accuracy == 0.0
        assert failure_reason is not None
        assert "Incorrect chunk count" in failure_reason


class TestReportGenerator:
    """Tests for the report generator functionality."""
    
    def test_generate_validation_report(self):
        """Test generating a validation report."""
        result = ValidationResult(
            query_id="query_001",
            retrieved_chunks=[],
            accuracy_score=0.75,
            correctness=True,
            comparison_details={"text_similarity": 0.75},
            failure_reason=None
        )
        
        report = generate_validation_report(
            validation_results=[result],
            total_queries=1,
            successful_queries=1,
            query_type_breakdown={"factual": {"accuracy": 100.0, "count": 1, "correct": 1}}
        )
        
        assert report.total_queries == 1
        assert report.successful_queries == 1
        assert report.accuracy_percentage == 100.0  # (1/1) * 100
        assert len(report.detailed_results) == 1
        assert isinstance(report.failure_analysis, dict)
        assert isinstance(report.query_type_breakdown, dict)
    
    def test_analyze_failures(self):
        """Test failure analysis function."""
        correct_result = ValidationResult(
            query_id="query_001",
            retrieved_chunks=[],
            accuracy_score=0.8,
            correctness=True,
            comparison_details={},
            failure_reason=None
        )
        
        failed_result = ValidationResult(
            query_id="query_002",
            retrieved_chunks=[],
            accuracy_score=0.2,
            correctness=False,
            comparison_details={},
            failure_reason="Content similarity below threshold"
        )
        
        failures = analyze_failures([correct_result, failed_result])
        
        assert failures["total_failures"] == 1
        assert failures["failure_percentage"] == 50.0  # 1 out of 2
        assert isinstance(failures["top_failure_reasons"], dict)


class TestValidationRunner:
    """Tests for the validation runner functionality."""
    
    def test_load_test_queries(self, tmp_path):
        """Test loading test queries from JSON file."""
        # Create a temporary JSON file with test queries
        queries_file = tmp_path / "test_queries.json"
        queries_data = {
            "queries": [
                {
                    "id": "factual_001",
                    "text": "What is the primary goal?",
                    "type": "factual",
                    "expected_section": "Introduction",
                    "metadata": {"difficulty": "easy"}
                }
            ]
        }
        
        with open(queries_file, 'w') as f:
            import json
            json.dump(queries_data, f)
        
        queries = load_test_queries(str(queries_file))
        
        assert len(queries) == 1
        assert queries[0].id == "factual_001"
        assert queries[0].text == "What is the primary goal?"
        assert queries[0].type == "factual"
        assert queries[0].expected_section == "Introduction"
        assert queries[0].metadata == {"difficulty": "easy"}