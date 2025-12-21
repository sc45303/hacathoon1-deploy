from fastapi import APIRouter, HTTPException, Depends
from app.models.agent import QueryRequest, QueryResponse
from app.services.agent_service import AgentService
# from app.core.auth import verify_api_key  # Assuming we have an auth module


router = APIRouter()


@router.post("/agent/query", response_model=QueryResponse)
async def agent_query(
    request: QueryRequest,
    # api_key: str = Depends(verify_api_key)  # Uncomment if authentication is needed
):
    """
    Agent Router Endpoint: Central decision-making endpoint that classifies and routes 
    queries to appropriate answering strategies
    """
    try:
        # Process the query using the agent service
        response = await AgentService.process_query(request)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")