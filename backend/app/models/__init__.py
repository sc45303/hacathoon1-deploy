"""
Pydantic Models for the RAG Chatbot Backend.

This module contains Pydantic models for data validation,
API request/response schemas, and configuration settings.
"""
from pydantic import BaseModel
from datetime import datetime
from typing import Literal, Optional


class HealthResponse(BaseModel):
    """
    Response model for the health check endpoint.

    Fields:
    - status: Health status (healthy, degraded, unhealthy)
    - message: Description of the health state
    - timestamp: ISO 8601 formatted timestamp
    """
    status: Literal["healthy", "degraded", "unhealthy"]
    message: str
    timestamp: datetime

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


# Future models will be added here as the application grows:
# - Request models for API payloads
# - Response models for API outputs
# - Data models for database interactions