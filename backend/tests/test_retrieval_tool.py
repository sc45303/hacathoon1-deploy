"""
Unit tests for the retrieval tool of the retrieval-enabled agent service.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.src.agent.retrieval_tool import RetrievalTool
from backend.src.agent.config import AgentConfig


class TestRetrievalTool:
    """Tests for the RetrievalTool class."""
    
    @pytest.fixture
    def config(self):
        """Create a mock configuration for testing."""
        config = Mock(spec=AgentConfig)
        config.cohere_api_key = "test-cohere-key"
        config.qdrant_url = "https://test-qdrant-url"
        config.qdrant_api_key = "test-qdrant-key"
        config.qdrant_collection_name = "test-collection"
        config.cohere_model = "embed-multilingual-v3.0"
        config.top_k_chunks = 5
        return config
    
    @pytest.fixture
    def retrieval_tool(self, config):
        """Create a retrieval tool instance for testing."""
        with patch('backend.src.agent.retrieval_tool.cohere.Client'), \
             patch('backend.src.agent.retrieval_tool.QdrantClient'):
            tool = RetrievalTool(config)
            return tool
    
    def test_initialization(self, config):
        """Test that the retrieval tool initializes correctly."""
        with patch('backend.src.agent.retrieval_tool.cohere.Client') as mock_cohere, \
             patch('backend.src.agent.retrieval_tool.QdrantClient') as mock_qdrant:
            tool = RetrievalTool(config)
            
            # Verify that clients were initialized
            mock_cohere.assert_called_once_with(config.cohere_api_key)
            mock_qdrant.assert_called_once_with(
                url=config.qdrant_url,
                api_key=config.qdrant_api_key,
                timeout=10
            )
            assert tool.config == config
    
    def test_generate_embedding(self, retrieval_tool):
        """Test that embedding generation works correctly."""
        test_text = "This is a test query"
        expected_embedding = [0.1, 0.2, 0.3]
        
        # Mock the Cohere client response
        retrieval_tool.cohere_client.embed.return_value = Mock()
        retrieval_tool.cohere_client.embed.return_value.embeddings = [expected_embedding]
        
        # Call the method
        result = retrieval_tool._generate_embedding(test_text)
        
        # Verify the result
        assert result == expected_embedding
        
        # Verify that the Cohere client was called correctly
        retrieval_tool.cohere_client.embed.assert_called_once_with(
            texts=[test_text],
            model=retrieval_tool.config.cohere_model,
            input_type="search_query"
        )
    
    def test_retrieve_chunks(self, retrieval_tool):
        """Test that chunk retrieval works correctly."""
        query = "What is the main topic?"
        mock_embedding = [0.1, 0.2, 0.3]
        mock_search_result = [
            Mock(id="chunk-1", payload={
                "text": "This is the first relevant chunk", 
                "url": "https://example.com/chunk1",
                "position": 1,
                "source_metadata": {}
            }, score=0.95),
            Mock(id="chunk-2", payload={
                "text": "This is the second relevant chunk", 
                "url": "https://example.com/chunk2",
                "position": 2,
                "source_metadata": {}
            }, score=0.87)
        ]
        
        # Mock internal methods
        retrieval_tool._generate_embedding = Mock(return_value=mock_embedding)
        retrieval_tool.qdrant_client.search.return_value = mock_search_result
        
        # Call the method
        result = retrieval_tool.retrieve_chunks(query)
        
        # Verify the results
        assert len(result) == 2
        assert result[0].content == "This is the first relevant chunk"
        assert result[0].url == "https://example.com/chunk1"
        assert result[0].relevance_score == 0.95
        assert result[1].content == "This is the second relevant chunk"
        assert result[1].url == "https://example.com/chunk2"
        assert result[1].relevance_score == 0.87
        
        # Verify that internal methods were called correctly
        retrieval_tool._generate_embedding.assert_called_once_with(query)
        retrieval_tool.qdrant_client.search.assert_called_once_with(
            collection_name=retrieval_tool.config.qdrant_collection_name,
            query_vector=mock_embedding,
            limit=retrieval_tool.config.top_k_chunks,
            with_payload=True,
            with_vectors=False
        )
    
    def test_retrieve_chunks_for_agent(self, retrieval_tool):
        """Test the agent interface for chunk retrieval."""
        query = "Test query"
        mock_chunks = [
            Mock(chunk_id="chunk-1", content="Content 1", url="url1", position=1, 
                 relevance_score=0.9, source_metadata={})
        ]
        
        # Mock the retrieve method
        retrieval_tool.retrieve_chunks = Mock(return_value=mock_chunks)
        
        # Call the method
        result = retrieval_tool.retrieve_chunks_for_agent(query)
        
        # Verify the result is a string representation of the chunks
        assert "chunk-1" in result
        assert "Content 1" in result
        
        retrieval_tool.retrieve_chunks.assert_called_once_with(query)


if __name__ == "__main__":
    pytest.main()