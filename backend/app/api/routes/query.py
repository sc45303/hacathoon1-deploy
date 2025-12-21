"""
Query API route for the RAG (Retrieval Augmented Generation) system.

This module defines the /query endpoint that allows users to submit
natural language questions and receive answers based on the book content.
"""
from typing import Dict, Any
import logging
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field

from app.rag.pipeline import RAGPipeline


logger = logging.getLogger(__name__)

# Create the router
router = APIRouter()

# Request model
class QueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000, description="The natural language question to answer")


# Response model
class SourceCitation(BaseModel):
    source_file: str
    chapter: str
    section: str


class QueryResponse(BaseModel):
    answer: str
    sources: list[SourceCitation]


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Handle a query request and return an answer based on book content.
    
    Args:
        request: QueryRequest containing the question
        
    Returns:
        QueryResponse with the answer and sources
    """
    try:
        logger.info(f"Received query: {request.question[:100]}...")
        
        # Initialize the RAG pipeline
        pipeline = RAGPipeline()
        
        # Process the query
        result = await pipeline.query(request.question)
        
        logger.info(f"Query processed successfully, response length: {len(result['answer'])}")
        
        # Format the response according to the API contract
        response = QueryResponse(
            answer=result['answer'],
            sources=[SourceCitation(**source) for source in result['sources']]
        )
        
        return response
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your question. Please try again."
        )