"""
Utility functions for the retrieval-enabled agent service.

This module provides common utility functions for logging, ID generation,
and other shared functionality across the agent components.
"""
import uuid
import logging
import time
from datetime import datetime
from typing import Dict, Any, Optional
from contextvars import ContextVar


# Context variables for request-specific data
request_id_var: ContextVar[Optional[str]] = ContextVar('request_id', default=None)
session_id_var: ContextVar[Optional[str]] = ContextVar('session_id', default=None)


def generate_id(prefix: str = "") -> str:
    """
    Generate a unique identifier with an optional prefix.
    
    Args:
        prefix: Optional prefix to add to the ID
        
    Returns:
        String identifier in the format "prefix-uuid" or just "uuid"
    """
    unique_id = str(uuid.uuid4())
    return f"{prefix}-{unique_id}" if prefix else unique_id


def setup_logging(log_level: str = "INFO") -> logging.Logger:
    """
    Set up logging configuration for the agent service.
    
    Args:
        log_level: Logging level (default: "INFO")
        
    Returns:
        Configured logger instance
    """
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    logger = logging.getLogger(__name__)
    return logger


def log_agent_interaction(step: str, data: Dict[str, Any], logger: logging.Logger) -> None:
    """
    Log an interaction in the Agent → Tool → Agent flow.
    
    Args:
        step: The step in the interaction flow (e.g., "agent_call", "tool_retrieval", "agent_response")
        data: Data associated with the interaction
        logger: Logger instance to use for logging
    """
    request_id = request_id_var.get()
    session_id = session_id_var.get()
    
    log_data = {
        "step": step,
        "timestamp": datetime.now().isoformat(),
        "request_id": request_id,
        "session_id": session_id,
        **data
    }
    
    logger.info(f"Agent interaction: {log_data}")


def time_function(func):
    """
    Decorator to time function execution.
    
    Args:
        func: The function to time
        
    Returns:
        Wrapper function that times execution
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time
    return wrapper


def validate_and_sanitize_input(text: str, max_length: int = 1000) -> str:
    """
    Validate and sanitize user input.
    
    Args:
        text: Input text to validate
        max_length: Maximum allowed length for the text
        
    Returns:
        Sanitized text
        
    Raises:
        ValueError: If the input fails validation
    """
    if not text or not text.strip():
        raise ValueError("Input text cannot be empty")
    
    if len(text) > max_length:
        raise ValueError(f"Input text exceeds maximum length of {max_length} characters")
    
    # Sanitize the text (basic sanitization)
    sanitized = text.strip()
    
    return sanitized


def format_timestamp(timestamp: datetime) -> str:
    """
    Format a datetime object as an ISO 8601 string.
    
    Args:
        timestamp: The datetime to format
        
    Returns:
        ISO 8601 formatted string
    """
    return timestamp.isoformat()


class Timer:
    """
    Context manager for timing code blocks.
    
    Example usage:
        with Timer() as timer:
            # code to time
        print(f"Execution time: {timer.elapsed:.2f} seconds")
    """
    def __init__(self):
        self.start_time = None
        self.end_time = None
        self.elapsed = None
    
    def __enter__(self):
        self.start_time = time.time()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.end_time = time.time()
        self.elapsed = self.end_time - self.start_time