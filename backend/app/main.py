from fastapi import FastAPI
from datetime import datetime
import os
from fastapi.middleware.cors import CORSMiddleware

from app.core.config import settings
from app.core.logging import get_logger
from app.models import HealthResponse
from app.api.routes.query import router as query_router
from app.api.routes.agent import router as agent_router

# Create the FastAPI application instance
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="Backend for RAG Chatbot integrated with Docusaurus-based book",
    debug=settings.debug,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, change this to your specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(query_router)
app.include_router(agent_router)

# Initialize logger
logger = get_logger(__name__)

@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot Backend"}

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify backend is operational.

    Returns a simple health status response to verify that the backend
    application is running correctly.
    """
    logger.info("Health check endpoint accessed")
    return HealthResponse(
        status="healthy",
        message="Backend is operational",
        timestamp=datetime.utcnow()
    )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host=settings.host,
        port=settings.port,
        reload=settings.debug
    )