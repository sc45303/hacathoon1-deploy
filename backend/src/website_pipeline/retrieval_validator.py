"""
Core validation logic for the retrieval pipeline.

This module handles the initialization of external services (Qdrant, Cohere),
embedding generation, and similarity search functionality needed for validation.
"""
import os
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import cohere
import logging
from datetime import datetime


# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)


class RetrievalValidator:
    """
    Core class for validating the retrieval pipeline.
    
    This class handles connecting to Qdrant and Cohere, generating embeddings,
    and performing similarity searches to validate the retrieval pipeline.
    """
    
    def __init__(self, collection_name: str = "reg-embedding", top_k: int = 5):
        """
        Initialize the retrieval validator.
        
        Args:
            collection_name: Name of the Qdrant collection to validate
            top_k: Number of top results to retrieve
        """
        self.collection_name = collection_name
        self.top_k = top_k
        
        # Initialize Cohere client (T005)
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")
        self.co = cohere.Client(api_key)

        # Initialize Qdrant client (T004)
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variables are not set")

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=10
        )
    
    def generate_embedding(self, text: str, model: str = 'embed-multilingual-v3.0', input_type: str = "search_query") -> List[float]:
        """
        Generate embedding for text using Cohere (T007).
        
        Args:
            text: Text to generate embedding for
            model: Cohere model to use for embedding
            input_type: Type of input for the model
            
        Returns:
            Embedding vector as a list of floats
        """
        try:
            response = self.co.embed(
                texts=[text],
                model=model,
                input_type=input_type
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            # T035: Handle edge case when Cohere embedding service is unavailable
            raise
    
    def similarity_search(self, query_embedding: List[float], top_k: Optional[int] = None) -> List[dict]:
        """
        Perform similarity search in Qdrant.
        
        Args:
            query_embedding: Embedding vector to search for
            top_k: Number of top results to retrieve (uses instance default if not specified)
            
        Returns:
            List of search results with payload and similarity scores
        """
        try:
            search_top_k = top_k or self.top_k

            search_result = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=search_top_k,
                with_payload=True,
                with_vectors=False
            )

            results = []
            for hit in search_result:
                result = {
                    "id": hit.id,
                    "content": hit.payload.get("text", ""),
                    "url": hit.payload.get("url", ""),
                    "position": hit.payload.get("position", 0),
                    "relevance_score": hit.score,
                    "source_metadata": hit.payload.get("source_metadata", {})
                }
                results.append(result)

            return results
        except Exception as e:
            logger.error(f"Error during similarity search: {e}")
            # T033: Handle edge case when Qdrant service is unavailable
            raise
    
    def validate_single_query(self, query_text: str, top_k: Optional[int] = None) -> List[dict]:
        """
        Validate a single query by generating embedding and searching for similar content.
        
        Args:
            query_text: Text to validate
            top_k: Number of top results to retrieve (uses instance default if not specified)
            
        Returns:
            List of retrieved chunks with metadata
        """
        # Generate embedding for the query (T007)
        query_embedding = self.generate_embedding(query_text)
        
        # Perform similarity search (T010)
        results = self.similarity_search(query_embedding, top_k)

        return results

    def validate_multiple_queries(self, queries: List[str], top_k: Optional[int] = None) -> List[List[dict]]:
        """
        Validate multiple queries in sequence (T012).

        Args:
            queries: List of query texts to validate
            top_k: Number of top results to retrieve (uses instance default if not specified)

        Returns:
            List of results for each query
        """
        all_results = []
        for query in queries:
            try:
                result = self.validate_single_query(query, top_k)
                # Validate that top 3-5 chunks are returned (T013)
                if not (3 <= len(result) <= 5):
                    logger.warning(f"Query '{query}' returned {len(result)} results, expected 3-5")
                all_results.append(result)
            except Exception as e:
                logger.error(f"Error validating query '{query}': {e}")
                # Return empty result for failed queries
                all_results.append([])

        return all_results