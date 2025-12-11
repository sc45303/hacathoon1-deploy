"""
Configuration management for the retrieval-enabled agent service.

This module handles loading and validating configuration from environment variables,
including API keys for external services like OpenAI, Qdrant, and Cohere.
"""
import os
from typing import Optional
from pydantic import BaseModel, Field


class AgentConfig(BaseModel):
    """
    Configuration for the retrieval-enabled agent service.
    
    Attributes:
        openai_api_key: API key for OpenAI services
        qdrant_url: URL for the Qdrant vector database
        qdrant_api_key: API key for Qdrant access
        cohere_api_key: API key for Cohere services
        qdrant_collection_name: Name of the collection in Qdrant (default: "reg-embedding")
        agent_model: OpenAI model to use for the agent (default: "gpt-4-1106-preview")
        cohere_model: Cohere model to use for embeddings (default: "embed-multilingual-v3.0")
        top_k_chunks: Number of top chunks to retrieve (default: 3-5 per requirements)
        timeout_seconds: Request timeout in seconds (default: 10 per requirements)
        log_level: Logging level (default: "INFO")
    """
    openai_api_key: str = Field(..., description="API key for OpenAI services")
    qdrant_url: str = Field(..., description="URL for the Qdrant vector database")
    qdrant_api_key: str = Field(..., description="API key for Qdrant access")
    cohere_api_key: str = Field(..., description="API key for Cohere services")
    qdrant_collection_name: str = Field(default="reg-embedding", description="Name of the collection in Qdrant")
    agent_model: str = Field(default="gpt-4-1106-preview", description="OpenAI model to use for the agent")
    cohere_model: str = Field(default="embed-multilingual-v3.0", description="Cohere model to use for embeddings")
    top_k_chunks: int = Field(default=5, description="Number of top chunks to retrieve (3-5 per requirements)")
    timeout_seconds: int = Field(default=10, description="Request timeout in seconds (10s per requirements)")
    log_level: str = Field(default="INFO", description="Logging level")

    @classmethod
    def from_env(cls):
        """
        Create a configuration instance from environment variables.
        
        Returns:
            AgentConfig: Configuration loaded from environment variables
        """
        return cls(
            openai_api_key=os.getenv("OPENAI_API_KEY"),
            qdrant_url=os.getenv("QDRANT_URL"),
            qdrant_api_key=os.getenv("QDRANT_API_KEY"),
            cohere_api_key=os.getenv("COHERE_API_KEY"),
            qdrant_collection_name=os.getenv("QDRANT_COLLECTION_NAME", "reg-embedding"),
            agent_model=os.getenv("AGENT_MODEL", "gpt-4-1106-preview"),
            cohere_model=os.getenv("COHERE_MODEL", "embed-multilingual-v3.0"),
            top_k_chunks=int(os.getenv("TOP_K_CHUNKS", "5")),
            timeout_seconds=int(os.getenv("TIMEOUT_SECONDS", "10")),
            log_level=os.getenv("LOG_LEVEL", "INFO")
        )

    def validate(self) -> list[str]:
        """
        Validate the configuration values.
        
        Returns:
            List of validation errors, empty if no errors
        """
        errors = []
        
        if not self.openai_api_key:
            errors.append("OPENAI_API_KEY environment variable is not set")
        if not self.qdrant_url:
            errors.append("QDRANT_URL environment variable is not set")
        if not self.qdrant_api_key:
            errors.append("QDRANT_API_KEY environment variable is not set")
        if not self.cohere_api_key:
            errors.append("COHERE_API_KEY environment variable is not set")
            
        if self.top_k_chunks < 1 or self.top_k_chunks > 10:
            errors.append(f"top_k_chunks should be between 1 and 10, got {self.top_k_chunks}")
        
        if self.timeout_seconds < 1 or self.timeout_seconds > 60:
            errors.append(f"timeout_seconds should be between 1 and 60, got {self.timeout_seconds}")
        
        return errors