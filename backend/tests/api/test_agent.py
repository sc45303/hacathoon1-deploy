import pytest
from fastapi.testclient import TestClient
from app.main import app
from app.models.agent import QueryRequest


client = TestClient(app)


class TestAgentEndpoint:
    """Integration tests for the agent endpoint."""
    
    def test_agent_endpoint_selected_text_mode(self):
        """Test the agent endpoint with selected text mode."""
        request_data = {
            "question": "What does this text mean?",
            "selected_text": "The main themes include good vs evil and friendship."
        }
        
        response = client.post("/agent/query", json=request_data)
        
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "mode" in data
        assert data["mode"] == "selected_text"
    
    def test_agent_endpoint_book_mode(self):
        """Test the agent endpoint with book mode (default)."""
        request_data = {
            "question": "What are the main themes in this book?"
        }
        
        response = client.post("/agent/query", json=request_data)
        
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "mode" in data
        assert data["mode"] == "book"
    
    def test_agent_endpoint_general_mode(self):
        """Test the agent endpoint with general mode."""
        request_data = {
            "question": "What is the capital of France?",
            "mode": "general"
        }
        
        response = client.post("/agent/query", json=request_data)
        
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "mode" in data
        assert data["mode"] == "general"
    
    def test_agent_endpoint_invalid_request(self):
        """Test the agent endpoint with invalid request data."""
        request_data = {
            "question": ""  # Empty question should fail validation
        }
        
        response = client.post("/agent/query", json=request_data)
        
        # Should return 422 for validation error
        assert response.status_code == 422
    
    def test_agent_endpoint_missing_question(self):
        """Test the agent endpoint with missing question."""
        request_data = {
            # Missing required question field
        }
        
        response = client.post("/agent/query", json=request_data)
        
        # Should return 422 for validation error
        assert response.status_code == 422