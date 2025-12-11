"""
Configuration class for validation parameters.

This module defines a configuration class that manages parameters for the validation process,
such as collection name, top-k results, accuracy thresholds, etc.
"""
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class ValidationConfig:
    """
    Configuration class for validation parameters.
    
    Attributes:
        collection_name: Name of the Qdrant collection to validate
        top_k: Number of top results to retrieve (default: 5)
        threshold: Minimum accuracy threshold (default: 0.9)
        query_types: List of query types to test (default: ['factual', 'conceptual', 'section-specific'])
        test_queries_file: Path to the file containing test queries (default: 'validation/test_queries.json')
        output_path: Path to save validation report (default: 'validation_report.json')
        output_format: Output format ('json' or 'text', default: 'json')
        config_path: Path to configuration file (default: '.env')
    """
    collection_name: str = "reg-embedding"
    top_k: int = 5
    threshold: float = 0.9
    query_types: List[str] = None  # Will be initialized in __post_init__
    test_queries_file: str = "validation/test_queries.json"
    output_path: str = "validation_report.json"
    output_format: str = "json"
    config_path: str = ".env"
    
    def __post_init__(self):
        """Initialize default values that can't be set directly in the dataclass."""
        if self.query_types is None:
            self.query_types = ["factual", "conceptual", "section-specific"]
    
    @classmethod
    def from_dict(cls, config_dict: dict):
        """Create a ValidationConfig instance from a dictionary."""
        # Create a copy to avoid modifying the original
        params = config_dict.copy()
        
        # Only include parameters that match the dataclass fields
        field_names = {field.name for field in cls.__dataclass_fields__.values()}
        filtered_params = {k: v for k, v in params.items() if k in field_names}
        
        return cls(**filtered_params)
    
    def validate(self) -> List[str]:
        """
        Validate the configuration values.
        
        Returns:
            List of validation errors, empty if no errors
        """
        errors = []
        
        if self.top_k <= 0:
            errors.append(f"top_k must be positive, got {self.top_k}")
        
        if not (0 <= self.threshold <= 1):
            errors.append(f"threshold must be between 0 and 1, got {self.threshold}")
        
        if not self.collection_name:
            errors.append("collection_name cannot be empty")
        
        if not self.test_queries_file:
            errors.append("test_queries_file cannot be empty")
        
        if self.output_format not in ["json", "text"]:
            errors.append(f"output_format must be 'json' or 'text', got {self.output_format}")
        
        if not self.query_types:
            errors.append("query_types cannot be empty")
        
        return errors