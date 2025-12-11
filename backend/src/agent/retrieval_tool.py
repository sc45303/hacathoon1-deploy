"""
Retrieval tool for the OpenAI agent.

This module implements a custom tool that allows the OpenAI agent to retrieve
relevant text chunks from Qdrant using Cohere embeddings.
"""
import logging
from typing import List, Dict, Any
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from backend.src.agent.config import AgentConfig
from backend.src.agent.models import RetrievedChunk
from backend.src.agent.utils import generate_id, log_agent_interaction


class RetrievalTool:
    """
    Custom tool for retrieving relevant text chunks from Qdrant.
    
    This tool integrates with Qdrant vector database and Cohere for generating
    embeddings to find the most relevant chunks based on a query.
    """
    
    def __init__(self, config: AgentConfig):
        """
        Initialize the retrieval tool with configuration.
        
        Args:
            config: Agent configuration containing API keys and settings
        """
        self.config = config
        
        # Initialize Cohere client
        self.cohere_client = cohere.Client(config.cohere_api_key)
        
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=10
        )
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
    def _generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using Cohere.
        
        Args:
            text: Text to generate embedding for
            
        Returns:
            Embedding vector as a list of floats
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=self.config.cohere_model,
                input_type="search_query"
            )
            return response.embeddings[0]
        except Exception as e:
            self.logger.error(f"Error generating embedding: {e}")
            raise
    
    def retrieve_chunks(self, query: str) -> List[RetrievedChunk]:
        """
        Retrieve the most relevant text chunks from Qdrant based on the query.
        
        Args:
            query: The query text to search for relevant chunks
            
        Returns:
            List of RetrievedChunk objects with the most relevant content
        """
        log_agent_interaction("tool_call_start", {"tool": "retrieval", "query": query}, self.logger)
        
        try:
            # Generate embedding for the query
            query_embedding = self._generate_embedding(query)
            
            # Search in Qdrant
            search_result = self.qdrant_client.search(
                collection_name=self.config.qdrant_collection_name,
                query_vector=query_embedding,
                limit=self.config.top_k_chunks,
                with_payload=True,
                with_vectors=False
            )
            
            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for hit in search_result:
                chunk = RetrievedChunk(
                    chunk_id=hit.id,
                    content=hit.payload.get("text", ""),
                    url=hit.payload.get("url", ""),
                    position=hit.payload.get("position", 0),
                    relevance_score=hit.score,
                    source_metadata=hit.payload.get("source_metadata", {})
                )
                retrieved_chunks.append(chunk)
            
            log_agent_interaction("tool_call_end", {
                "tool": "retrieval", 
                "query": query, 
                "chunks_found": len(retrieved_chunks),
                "top_score": retrieved_chunks[0].relevance_score if retrieved_chunks else 0.0
            }, self.logger)
            
            return retrieved_chunks
            
        except Exception as e:
            self.logger.error(f"Error during retrieval: {e}")
            log_agent_interaction("tool_call_error", {
                "tool": "retrieval", 
                "query": query, 
                "error": str(e)
            }, self.logger)
            raise
    
    def retrieve_chunks_for_agent(self, query: str) -> str:
        """
        Interface method for the OpenAI agent to call this tool.
        
        Args:
            query: The query text to search for relevant chunks
            
        Returns:
            JSON string of retrieved chunks that can be parsed by the agent
        """
        try:
            chunks = self.retrieve_chunks(query)
            
            # Format chunks for agent consumption
            formatted_chunks = []
            for chunk in chunks:
                formatted_chunks.append({
                    "chunk_id": chunk.chunk_id,
                    "content": chunk.content,
                    "url": chunk.url,
                    "position": chunk.position,
                    "relevance_score": chunk.relevance_score,
                    "source_metadata": chunk.source_metadata
                })
            
            return str(formatted_chunks)
        except Exception as e:
            self.logger.error(f"Error in agent interface: {e}")
            return f"Error retrieving chunks: {str(e)}"