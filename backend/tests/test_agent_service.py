import pytest
from unittest.mock import AsyncMock, patch
from app.models.agent import QueryRequest, QueryResponse, Source
from app.services.agent_service import AgentService


class TestAgentService:
    """Unit tests for the AgentService class."""
    
    def test_classify_query_selected_text_mode(self):
        """Test that queries with selected_text are classified as selected_text mode."""
        request = QueryRequest(
            question="What does this text mean?",
            selected_text="The main themes include good vs evil and friendship."
        )
        
        mode = AgentService.classify_query(request)
        assert mode == "selected_text"
    
    def test_classify_query_general_mode_explicit(self):
        """Test that queries with mode=general are classified as general mode."""
        request = QueryRequest(
            question="What is the capital of France?",
            mode="general"
        )
        
        mode = AgentService.classify_query(request)
        assert mode == "general"
    
    def test_classify_query_book_mode_default(self):
        """Test that queries without selected_text or explicit mode default to book mode."""
        request = QueryRequest(
            question="What are the main themes in this book?"
        )
        
        mode = AgentService.classify_query(request)
        assert mode == "book"
    
    @pytest.mark.asyncio
    async def test_handle_selected_text_mode(self):
        """Test handling of selected text mode queries."""
        request = QueryRequest(
            question="What does this text mean?",
            selected_text="The main themes include good vs evil and friendship."
        )
        
        # Mock the OpenAI client response
        with patch('app.services.agent_service.client') as mock_client:
            mock_response = AsyncMock()
            mock_response.choices = [AsyncMock()]
            mock_response.choices[0].message = AsyncMock()
            mock_response.choices[0].message.content = "This text discusses the themes of good vs evil and friendship."
            
            mock_client.chat.completions.create.return_value = mock_response
            
            result = await AgentService.handle_selected_text_mode(request)
            
            assert result.mode == "selected_text"
            assert result.answer is not None
            assert len(result.sources) == 1
            assert result.sources[0].document_id == "selected_text"
    
    @pytest.mark.asyncio
    async def test_handle_book_rag_mode(self):
        """Test handling of book RAG mode queries."""
        request = QueryRequest(
            question="What are the main themes in this book?"
        )
        
        # Mock the RAGPipeline
        with patch('app.services.agent_service.RAGPipeline') as mock_pipeline_class:
            mock_pipeline_instance = AsyncMock()
            mock_pipeline_instance.query.return_value = {
                'answer': 'The main themes are good vs evil and friendship.',
                'sources': [
                    {
                        'source_file': 'book1.txt',
                        'chapter': 'Chapter 1',
                        'section': 'Introduction'
                    }
                ]
            }
            mock_pipeline_class.return_value = mock_pipeline_instance
            
            result = await AgentService.handle_book_rag_mode(request)
            
            assert result.mode == "book"
            assert result.answer is not None
            assert len(result.sources) == 1
    
    @pytest.mark.asyncio
    async def test_handle_general_mode(self):
        """Test handling of general knowledge mode queries."""
        request = QueryRequest(
            question="What is the capital of France?"
        )
        
        # Mock the OpenAI client response
        with patch('app.services.agent_service.client') as mock_client:
            mock_response = AsyncMock()
            mock_response.choices = [AsyncMock()]
            mock_response.choices[0].message = AsyncMock()
            mock_response.choices[0].message.content = "The capital of France is Paris."
            
            mock_client.chat.completions.create.return_value = mock_response
            
            result = await AgentService.handle_general_mode(request)
            
            assert result.mode == "general"
            assert result.answer is not None
            assert len(result.sources) == 0
    
    @pytest.mark.asyncio
    async def test_process_query_routing(self):
        """Test that process_query correctly routes to the appropriate handler."""
        # Test selected text routing
        selected_request = QueryRequest(
            question="What does this mean?",
            selected_text="Some selected text"
        )
        
        with patch('app.services.agent_service.AgentService.handle_selected_text_mode') as mock_handler:
            mock_response = QueryResponse(
                answer="Test answer",
                sources=[],
                mode="selected_text"
            )
            mock_handler.return_value = mock_response
            
            result = await AgentService.process_query(selected_request)
            
            mock_handler.assert_called_once_with(selected_request)
            assert result.mode == "selected_text"