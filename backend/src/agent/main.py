"""
Main FastAPI application for the retrieval-enabled agent service.

This module defines the FastAPI application with all required endpoints
for the agent service.
"""
import os
import logging
from datetime import datetime
from typing import Dict, Any, Optional
from fastapi import FastAPI, HTTPException, Request, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from backend.src.agent.config import AgentConfig
from backend.src.agent.agent_service import AgentService
from backend.src.agent.models import (
    AskRequest, AskResponse, HealthResponse, 
    AgentRunRequest, AgentRunResponse, ToolCallInfo,
    RetrievedChunk
)
from backend.src.agent.utils import generate_id, format_timestamp, Timer, request_id_var, session_id_var


# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Create global agent service instance
agent_service: Optional[AgentService] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for the FastAPI application.
    Initializes global resources on startup and cleans up on shutdown.
    """
    global agent_service

    # Load configuration
    config = AgentConfig.from_env()
    errors = config.validate()
    if errors:
        raise RuntimeError(f"Configuration errors: {errors}")

    # Initialize agent service
    agent_service = AgentService(config)

    yield  # Application runs during this period

    # Cleanup (if needed)
    agent_service = None


# Create FastAPI app with lifespan
app = FastAPI(
    title="Retrieval-Enabled Agent API",
    description="An API for asking questions about book content using a retrieval-augmented agent",
    version="1.0.0",
    lifespan=lifespan
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def add_request_id(request: Request, call_next):
    """
    Middleware to add a unique request ID to each request.
    """
    request_id = generate_id("req")
    request_id_var.set(request_id)
    
    # Add session ID if provided in headers
    session_id = request.headers.get("X-Session-ID")
    if session_id:
        session_id_var.set(session_id)
    else:
        session_id = generate_id("sess")
        session_id_var.set(session_id)
    
    response = await call_next(request)
    response.headers["X-Request-ID"] = request_id
    response.headers["X-Session-ID"] = session_id
    
    return response


@app.post("/ask", response_model=AskResponse)
@limiter.limit("100/minute")  # 100 requests per minute per IP
async def ask_question(request: AskRequest) -> AskResponse:
    """
    Endpoint to ask questions about the book content.

    Args:
        request: AskRequest containing the query and optional session/user IDs

    Returns:
        AskResponse with the answer and sources
    """
    global agent_service
    
    if agent_service is None:
        raise HTTPException(status_code=500, detail="Agent service not initialized")
    
    try:
        # Set context variables if not already set by middleware
        if not request_id_var.get():
            request_id = generate_id("req")
            request_id_var.set(request_id)
        if not session_id_var.get():
            session_id = request.session_id or generate_id("sess")
            session_id_var.set(session_id)
        
        # Process the question with the agent
        with Timer() as timer:
            agent_response = agent_service.ask_question(
                query=request.query,
                thread_id=request.session_id
            )
        
        # Format retrieved sources
        sources = [
            RetrievedChunk(
                chunk_id=chunk.chunk_id,
                content=chunk.content,
                url=chunk.url,
                position=chunk.position,
                relevance_score=chunk.relevance_score,
                source_metadata=chunk.source_metadata
            )
            for chunk in agent_response.chunks_used
        ]
        
        # Create the response
        response = AskResponse(
            response_id=agent_response.response_id,
            answer=agent_response.content,
            sources=sources,
            query_id=agent_response.query_id,
            timestamp=format_timestamp(agent_response.timestamp),
            confidence_score=agent_response.confidence_score
        )
        
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing question: {str(e)}")


@app.get("/health", response_model=HealthResponse)
@limiter.limit("200/minute")  # 200 health check requests per minute per IP
async def health_check() -> HealthResponse:
    """
    Endpoint to check the health of the agent service and its dependencies.

    Returns:
        HealthResponse with status information
    """
    global agent_service
    
    if agent_service is None:
        raise HTTPException(status_code=500, detail="Agent service not initialized")
    
    # Perform health checks
    health_info = agent_service.check_health()
    
    response = HealthResponse(
        status=health_info["status"],
        timestamp=format_timestamp(datetime.now()),
        services=health_info["services"],
        details={
            "uptime_seconds": 0,  # Calculate uptime in a real implementation
            "active_sessions": 0,  # Track active sessions in a real implementation
            "version": "1.0.0"
        }
    )
    
    return response


@app.post("/agent/run", response_model=AgentRunResponse)
@limiter.limit("50/minute")  # 50 agent run requests per minute per IP
async def run_agent(request: AgentRunRequest) -> AgentRunResponse:
    """
    Endpoint to programmatically execute an agent run with custom instructions.

    Args:
        request: AgentRunRequest containing instructions and parameters

    Returns:
        AgentRunResponse with the agent's output
    """
    global agent_service
    
    if agent_service is None:
        raise HTTPException(status_code=500, detail="Agent service not initialized")
    
    try:
        # Set context variables if not already set by middleware
        if not request_id_var.get():
            request_id = generate_id("req")
            request_id_var.set(request_id)
        if not session_id_var.get():
            session_id = request.session_id or generate_id("sess")
            session_id_var.set(session_id)
        
        # Run the agent with custom instructions
        with Timer() as timer:
            agent_response = agent_service.run_agent_with_instructions(
                instructions=request.instructions,
                input_data=request.input_data
            )
        
        # Create response with mock tool calls (since we're not capturing actual tool calls in this implementation)
        # In a more sophisticated implementation, we'd track actual tool calls
        tool_calls = [
            ToolCallInfo(
                tool_name="retrieval_tool",
                input={"query": request.instructions},
                output={"chunks_retrieved": len(agent_response.chunks_used)}
            )
        ]
        
        response = AgentRunResponse(
            run_id=agent_response.response_id,
            status="completed",
            output=agent_response.content,
            timestamp=format_timestamp(agent_response.timestamp),
            execution_time_ms=int(timer.elapsed * 1000),
            tool_calls=tool_calls
        )
        
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error running agent: {str(e)}")


# Error handlers
@app.exception_handler(404)
async def not_found_handler(request: Request, exc: HTTPException):
    return {"error": "Endpoint not found", "error_code": "NOT_FOUND"}


@app.exception_handler(500)
async def internal_error_handler(request: Request, exc: HTTPException):
    return {"error": "Internal server error", "error_code": "INTERNAL_ERROR", "timestamp": format_timestamp(datetime.now())}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)