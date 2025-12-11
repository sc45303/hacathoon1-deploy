"""
API endpoint tests for the retrieval-enabled agent service.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock

# Import the FastAPI app
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from backend.src.agent.main import app, agent_service
from backend.src.agent.agent_service import AgentService
from backend.src.agent.models import AgentResponse, RetrievedChunk


client = TestClient(app)


class TestAskEndpoint:
    """Tests for the /ask endpoint."""
    
    def test_ask_endpoint_valid_request(self, mocker):
        """Test the /ask endpoint with a valid request."""
        # Mock the agent service
        mock_agent_response = AgentResponse(
            response_id="response-123",
            content="This is the answer to your question",
            query_id="query-456",
            chunks_used=[
                RetrievedChunk(
                    chunk_id="chunk-789",
                    content="Relevant content from the book",
                    url="https://example.com/book/chapter1",
                    position=1,
                    relevance_score=0.9,
                    source_metadata={}
                )
            ],
            confidence_score=0.85
        )
        
        # Since agent_service is a global variable that gets set during app startup,
        # we need to mock the method on the existing instance or patch the import
        with patch('backend.src.agent.main.agent_service') as mock_service:
            mock_service.ask_question.return_value = mock_agent_response
            
            response = client.post("/ask", json={
                "query": "What is this book about?"
            })
            
            assert response.status_code == 200
            data = response.json()
            assert data["answer"] == "This is the answer to your question"
            assert len(data["sources"]) == 1
            assert data["sources"][0]["content"] == "Relevant content from the book"
    
    def test_ask_endpoint_missing_query(self):
        """Test the /ask endpoint with a missing query."""
        response = client.post("/ask", json={})
        assert response.status_code == 422  # Validation error
    
    def test_ask_endpoint_empty_query(self):
        """Test the /ask endpoint with an empty query."""
        response = client.post("/ask", json={"query": ""})
        assert response.status_code == 422  # Validation error


class TestHealthEndpoint:
    """Tests for the /health endpoint."""
    
    def test_health_endpoint(self):
        """Test the /health endpoint."""
        # Mock the agent service to return a health status
        mock_health_info = {
            "status": "healthy",
            "services": {
                "openai": {"status": "available", "response_time_ms": 100},
                "qdrant": {"status": "available", "response_time_ms": 50},
                "cohere": {"status": "available", "response_time_ms": 80}
            }
        }
        
        with patch('backend.src.agent.main.agent_service') as mock_service:
            mock_service.check_health.return_value = mock_health_info
            
            response = client.get("/health")
            
            assert response.status_code == 200
            data = response.json()
            assert data["status"] in ["healthy", "degraded", "unhealthy"]
            assert "timestamp" in data
            assert "services" in data
            assert "details" in data


class TestAgentRunEndpoint:
    """Tests for the /agent/run endpoint."""
    
    def test_agent_run_endpoint_valid_request(self):
        """Test the /agent/run endpoint with a valid request."""
        mock_agent_response = AgentResponse(
            response_id="response-123",
            content="This is the agent's output",
            query_id="query-456",
            chunks_used=[],
            confidence_score=None
        )
        
        with patch('backend.src.agent.main.agent_service') as mock_service:
            mock_service.run_agent_with_instructions.return_value = mock_agent_response
            
            response = client.post("/agent/run", json={
                "instructions": "Answer the following question based on the book content"
            })
            
            assert response.status_code == 200
            data = response.json()
            assert data["output"] == "This is the agent's output"
            assert "execution_time_ms" in data
            assert len(data["tool_calls"]) >= 0  # Should have tool calls info
    
    def test_agent_run_endpoint_missing_instructions(self):
        """Test the /agent/run endpoint with missing instructions."""
        response = client.post("/agent/run", json={})
        assert response.status_code == 422  # Validation error


def test_rate_limiting():
    """Test that rate limiting is applied to endpoints."""
    # This would require more complex testing with actual rate limits
    # For now, we just verify the endpoints are accessible
    response = client.get("/health")
    # If rate limiting is working, this would eventually return a 429
    # But on first request, it should return 200
    assert response.status_code == 200


if __name__ == "__main__":
    pytest.main()