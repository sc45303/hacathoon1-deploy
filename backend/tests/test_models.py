"""
Unit tests for the data models of the retrieval-enabled agent service.
"""
import pytest
from datetime import datetime
from backend.src.agent.models import (
    UserQuery, RetrievedChunk, AgentResponse, 
    AgentSession, AgentSessionStatus, APIRequest, APIResponse,
    AskRequest, AskResponse, HealthResponse, AgentRunRequest, AgentRunResponse
)


class TestUserQuery:
    """Tests for the UserQuery model."""
    
    def test_user_query_creation(self):
        """Test creating a UserQuery instance."""
        query = UserQuery(
            query_id="test-query-1",
            text="What is this book about?",
            user_id="user-123",
            session_id="session-456"
        )
        
        assert query.query_id == "test-query-1"
        assert query.text == "What is this book about?"
        assert query.user_id == "user-123"
        assert query.session_id == "session-456"
        assert isinstance(query.timestamp, datetime)


class TestRetrievedChunk:
    """Tests for the RetrievedChunk model."""
    
    def test_retrieved_chunk_creation(self):
        """Test creating a RetrievedChunk instance."""
        chunk = RetrievedChunk(
            chunk_id="chunk-123",
            content="This is the content of the retrieved chunk",
            url="https://example.com/book/chapter1",
            position=1,
            relevance_score=0.95,
            source_metadata={"author": "Test Author", "date": "2023-01-01"}
        )
        
        assert chunk.chunk_id == "chunk-123"
        assert chunk.content == "This is the content of the retrieved chunk"
        assert chunk.url == "https://example.com/book/chapter1"
        assert chunk.position == 1
        assert chunk.relevance_score == 0.95
        assert chunk.source_metadata == {"author": "Test Author", "date": "2023-01-01"}


class TestAgentResponse:
    """Tests for the AgentResponse model."""
    
    def test_agent_response_creation(self):
        """Test creating an AgentResponse instance."""
        chunk = RetrievedChunk(
            chunk_id="chunk-123",
            content="This is the content of the retrieved chunk",
            url="https://example.com/book/chapter1",
            position=1,
            relevance_score=0.95,
            source_metadata={}
        )
        
        response = AgentResponse(
            response_id="response-456",
            content="This is the agent's answer",
            query_id="query-789",
            chunks_used=[chunk],
            confidence_score=0.85
        )
        
        assert response.response_id == "response-456"
        assert response.content == "This is the agent's answer"
        assert response.query_id == "query-789"
        assert len(response.chunks_used) == 1
        assert response.chunks_used[0].chunk_id == "chunk-123"
        assert response.confidence_score == 0.85


class TestAgentSession:
    """Tests for the AgentSession model."""
    
    def test_agent_session_creation(self):
        """Test creating an AgentSession instance."""
        session = AgentSession(
            session_id="session-111",
            status=AgentSessionStatus.ACTIVE,
            history=[{"query": "Hello", "response": "Hi"}]
        )
        
        assert session.session_id == "session-111"
        assert session.status == AgentSessionStatus.ACTIVE
        assert session.history == [{"query": "Hello", "response": "Hi"}]
        assert isinstance(session.created_at, datetime)
        assert isinstance(session.last_interaction, datetime)


class TestAskRequest:
    """Tests for the AskRequest model."""
    
    def test_ask_request_creation(self):
        """Test creating an AskRequest instance."""
        request = AskRequest(
            query="What is the main concept?",
            session_id="session-222",
            user_id="user-333"
        )
        
        assert request.query == "What is the main concept?"
        assert request.session_id == "session-222"
        assert request.user_id == "user-333"


class TestAskResponse:
    """Tests for the AskResponse model."""
    
    def test_ask_response_creation(self):
        """Test creating an AskResponse instance."""
        chunk = RetrievedChunk(
            chunk_id="chunk-444",
            content="Sample content",
            url="https://example.com",
            position=1,
            relevance_score=0.8,
            source_metadata={}
        )
        
        response = AskResponse(
            response_id="response-555",
            answer="The answer is here",
            sources=[chunk],
            query_id="query-666",
            timestamp=datetime.now().isoformat(),
            confidence_score=0.75
        )
        
        assert response.response_id == "response-555"
        assert response.answer == "The answer is here"
        assert len(response.sources) == 1
        assert response.query_id == "query-666"
        assert response.confidence_score == 0.75


if __name__ == "__main__":
    pytest.main()