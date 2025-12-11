"""
Agent functionality tests for the retrieval-enabled agent service.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.src.agent.agent_service import AgentService
from backend.src.agent.config import AgentConfig
from backend.src.agent.models import RetrievedChunk, AgentResponse
import openai


class TestAgentService:
    """Tests for the AgentService class."""
    
    @pytest.fixture
    def config(self):
        """Create a mock configuration for testing."""
        config = Mock(spec=AgentConfig)
        config.openai_api_key = "test-openai-key"
        config.cohere_api_key = "test-cohere-key"
        config.qdrant_url = "https://test-qdrant-url"
        config.qdrant_api_key = "test-qdrant-key"
        config.qdrant_collection_name = "test-collection"
        config.cohere_model = "embed-multilingual-v3.0"
        config.agent_model = "gpt-4-1106-preview"
        config.top_k_chunks = 5
        config.timeout_seconds = 10
        config.log_level = "INFO"
        return config
    
    @pytest.fixture
    def agent_service(self, config):
        """Create an agent service instance for testing."""
        # Mock OpenAI clients to avoid actual API calls
        with patch('backend.src.agent.agent_service.openai.beta.assistants.create') as mock_assistant_create, \
             patch('backend.src.agent.agent_service.openai.beta.threads.create') as mock_thread_create:
            
            # Set up mock assistant
            mock_assistant = Mock()
            mock_assistant.id = "test-assistant-id"
            mock_assistant_create.return_value = mock_assistant
            
            # Set up mock thread
            mock_thread = Mock()
            mock_thread.id = "test-thread-id"
            mock_thread_create.return_value = mock_thread
            
            # Mock the retrieval tool separately
            with patch('backend.src.agent.agent_service.RetrievalTool'):
                service = AgentService(config)
                
                # Set the assistant ID directly since it was mocked
                service.assistant = mock_assistant
                
                return service
    
    def test_initialization(self, config):
        """Test that the agent service initializes correctly."""
        with patch('backend.src.agent.agent_service.openai.beta.assistants.create') as mock_assistant_create, \
             patch('backend.src.agent.agent_service.RetrievalTool') as mock_retrieval_tool:
            
            mock_assistant = Mock()
            mock_assistant.id = "test-assistant-id"
            mock_assistant_create.return_value = mock_assistant
            
            service = AgentService(config)
            
            # Verify the assistant was created with correct parameters
            mock_assistant_create.assert_called_once()
            call_args = mock_assistant_create.call_args
            assert call_args.kwargs['model'] == config.agent_model
            
            # Verify the retrieval tool was initialized
            mock_retrieval_tool.assert_called_once_with(config)
            
            assert service.config == config
            assert service.assistant.id == "test-assistant-id"
    
    def test_create_thread(self, agent_service):
        """Test that thread creation works correctly."""
        with patch('backend.src.agent.agent_service.openai.beta.threads.create') as mock_create:
            mock_thread = Mock()
            mock_thread.id = "new-thread-id"
            mock_create.return_value = mock_thread
            
            thread_id = agent_service.create_thread()
            
            mock_create.assert_called_once()
            assert thread_id == "new-thread-id"
    
    def test_ask_question(self, agent_service):
        """Test the ask_question method."""
        # Mock the thread creation and interaction
        with patch('backend.src.agent.agent_service.openai.beta.threads.create') as mock_thread_create, \
             patch('backend.src.agent.agent_service.openai.beta.threads.messages') as mock_messages, \
             patch('backend.src.agent.agent_service.openai.beta.threads.runs') as mock_runs, \
             patch('backend.src.agent.agent_service.openai.beta.threads.runs.steps') as mock_steps:
            
            # Setup mocks
            mock_thread = Mock()
            mock_thread.id = "test-thread-123"
            mock_thread_create.create.return_value = mock_thread
            
            # Setup run mock
            mock_run = Mock()
            mock_run.id = "test-run-123"
            mock_run.status = "completed"
            mock_runs.create.return_value = mock_run
            mock_runs.retrieve.return_value = mock_run
            
            # Setup message mock
            mock_message_content = Mock()
            mock_message_content.text.value = "This is the agent's response to your query."
            mock_message = Mock()
            mock_message.content = [mock_message_content]
            mock_messages.list.return_value = Mock(data=[mock_message])
            
            # Setup steps mock
            mock_steps.list.return_value = Mock(data=[])
            
            # Mock the retrieval tool
            mock_chunk = RetrievedChunk(
                chunk_id="chunk-1",
                content="Relevant content from the book",
                url="https://example.com/book",
                position=1,
                relevance_score=0.9,
                source_metadata={}
            )
            agent_service.retrieval_tool.retrieve_chunks = Mock(return_value=[mock_chunk])
            
            # Call the method
            result = agent_service.ask_question("What is this book about?")
            
            # Verify the result
            assert isinstance(result, AgentResponse)
            assert result.content == "This is the agent's response to your query."
            assert len(result.chunks_used) == 1
            assert result.chunks_used[0].content == "Relevant content from the book"
            
            # Verify the API calls
            mock_thread_create.create.assert_called_once()
            mock_messages.create.assert_called_once()
            mock_runs.create.assert_called_once()
    
    def test_run_agent_with_instructions(self, agent_service):
        """Test the run_agent_with_instructions method."""
        with patch('backend.src.agent.agent_service.openai.beta.threads.create') as mock_thread_create, \
             patch('backend.src.agent.agent_service.openai.beta.threads.messages') as mock_messages, \
             patch('backend.src.agent.agent_service.openai.beta.threads.runs') as mock_runs:
            
            # Setup mocks
            mock_thread = Mock()
            mock_thread.id = "test-thread-456"
            mock_thread_create.create.return_value = mock_thread
            
            # Setup run mock
            mock_run = Mock()
            mock_run.id = "test-run-456"
            mock_run.status = "completed"
            mock_runs.create.return_value = mock_run
            mock_runs.retrieve.return_value = mock_run
            
            # Setup message mock
            mock_message_content = Mock()
            mock_message_content.text.value = "This is the agent's response to the instructions."
            mock_message = Mock()
            mock_message.content = [mock_message_content]
            mock_messages.list.return_value = Mock(data=[mock_message])
            
            # Call the method
            result = agent_service.run_agent_with_instructions("Summarize the book")
            
            # Verify the result
            assert isinstance(result, AgentResponse)
            assert result.content == "This is the agent's response to the instructions."
            
            # Verify the API calls
            mock_thread_create.create.assert_called_once()
            mock_messages.create.assert_called_once()
            mock_runs.create.assert_called_once()
    
    def test_check_health(self, agent_service):
        """Test the check_health method."""
        # Mock the retrieval tool health checks
        agent_service.retrieval_tool.retrieve_chunks = Mock(return_value=[])
        agent_service.retrieval_tool._generate_embedding = Mock(return_value=[0.1, 0.2, 0.3])
        
        with patch('backend.src.agent.agent_service.openai.models.list') as mock_models_list:
            mock_models_list.return_value = Mock()
            
            health_result = agent_service.check_health()
            
            # Verify structure of response
            assert "status" in health_result
            assert "services" in health_result
            assert isinstance(health_result["services"], dict)
            assert "openai" in health_result["services"]
            assert "qdrant" in health_result["services"]
            assert "cohere" in health_result["services"]
            
            # Verify each service has required fields
            for service_name, service_info in health_result["services"].items():
                assert "status" in service_info
                assert "response_time_ms" in service_info


if __name__ == "__main__":
    pytest.main()