"""
Vector retrieval module for the RAG (Retrieval Augmented Generation) system.

This module provides async methods to perform similarity search in Qdrant
and retrieve the most relevant content chunks for a given question.
"""
from typing import List, Dict, Any
import logging

from app.db.qdrant import QdrantDB
from app.utils.embeddings import get_embedding
from app.core.config import settings

logger = logging.getLogger(__name__)


class VectorRetriever:
    """
    Handles vector retrieval from Qdrant based on semantic similarity.
    Now returns full payload including 'content' so pipeline can use it directly.
    """
    
    def __init__(self):
        self.qdrant_db = QdrantDB()
        self._collection_name = settings.QDRANT_COLLECTION_NAME

    async def retrieve_similar_chunks(
        self, 
        query_text: str, 
        top_k: int = 5,
        min_score: float = 0.0
    ) -> List[Dict[str, Any]]:
        """
        Perform similarity search in Qdrant and return results with full payload.
        
        Returns:
            List of dicts with:
            - id: Qdrant point ID
            - score: similarity score
            - payload: full payload including 'content', 'source_file', 'chapter', etc.
        """
        try:
            # 1. Get embedding for the query
            query_embedding = await get_embedding(query_text)

            # 2. Search in Qdrant
            search_results = await self.qdrant_db.search_similar(
                query_embedding=query_embedding,
                top_k=top_k
            )

            # 3. Return clean results with full payload (important for content!)
            normalized_results = []
            for result in search_results:
                if result.get("score", 0) >= min_score:
                    normalized_results.append({
                        "id": result["id"],              # Qdrant point UUID
                        "score": result["score"],
                        "payload": result.get("payload", {})  # ← YE SABSE IMPORTANT – pura payload including content
                    })

            logger.info(f"Retrieved {len(normalized_results)} relevant chunks for query: {query_text}")
            return normalized_results
            
        except Exception as e:
            logger.error(f"Error during vector retrieval: {e}")
            return []

    async def retrieve_by_chunk_ids(self, chunk_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve specific points by their Qdrant IDs (not Neon chunk_id).
        """
        try:
            results = []
            for chunk_id in chunk_ids:
                vector_data = await self.qdrant_db.get_vector_by_id(chunk_id)
                if vector_data:
                    results.append(vector_data)
            
            logger.info(f"Retrieved {len(results)} vectors by ID")
            return results
        except Exception as e:
            logger.error(f"Error retrieving vectors by ID: {e}")
            return []