"""
Qdrant Cloud connector module for the RAG chatbot backend.
"""
from typing import Dict, List, Optional, Any, Tuple
from uuid import uuid4
from datetime import datetime
import logging

from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models

from app.core.config import settings

logger = logging.getLogger(__name__)


class QdrantDB:
    def __init__(self):
        self._client: Optional[AsyncQdrantClient] = None
        self._url = settings.QDRANT_URL
        self._api_key = settings.QDRANT_API_KEY
        self._collection_name = settings.QDRANT_COLLECTION_NAME

    async def _get_client(self) -> AsyncQdrantClient:
        if self._client is None:
            if not all([self._url, self._api_key, self._collection_name]):
                raise ValueError("Missing Qdrant environment variables")

            self._client = AsyncQdrantClient(
                url=self._url,
                api_key=self._api_key,
                prefer_grpc=False
            )
            logger.info("QdrantDB client created")

        return self._client

    # -----------------------------
    # Collection
    # -----------------------------

    async def ensure_collection(self) -> bool:
        client = await self._get_client()

        collections = await client.get_collections()
        names = [c.name for c in collections.collections]

        if self._collection_name in names:
            return True

        await client.create_collection(
            collection_name=self._collection_name,
            vectors_config=models.VectorParams(
                size=1536,
                distance=models.Distance.COSINE
            )
        )
        logger.info(f"Created collection: {self._collection_name}")
        return True

    # -----------------------------
    # Upload
    # -----------------------------

    async def upload_vector(self, embedding: List[float], payload: Dict[str, Any]) -> bool:
        client = await self._get_client()
        await self.ensure_collection()

        await client.upsert(
            collection_name=self._collection_name,
            points=[
                models.PointStruct(
                    id=str(uuid4()),
                    vector=embedding,
                    payload=payload
                )
            ]
        )
        return True

    # -----------------------------
    # 🔥 FIXED SEARCH (IMPORTANT)
    # -----------------------------

    async def search_similar(
        self,
        query_embedding: List[float],
        top_k: int = 5
    ) -> List[Dict[str, Any]]:

        client = await self._get_client()

        try:
            results = await client.query_points(
                collection_name=self._collection_name,
                query=query_embedding,          # Direct list of floats
                limit=top_k,
                with_payload=True,              # Returns the payload (your chunks/metadata)
                # with_vectors=False,           # Optional: skip returning vectors to save bandwidth
            )

            output = []
            for point in results.points:        # results.points is the list of ScoredPoint
                output.append({
                    "id": point.id,
                    "score": point.score,
                    "payload": point.payload
                })

            logger.info(f"Qdrant returned {len(output)} results")
            return output

        except Exception as e:
            logger.error(f"Qdrant similarity search failed: {e}")
            return []
    # -----------------------------
    # Utilities
    # -----------------------------

    async def validate_connection(self) -> bool:
        """Validate connection to Qdrant."""
        try:
            client = await self._get_client()
            # Try to get collections to verify connection
            await client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant connection: {e}")
            return False

    async def health_check(self) -> Dict[str, Any]:
        """Perform health check on Qdrant."""
        try:
            client = await self._get_client()

            # Get collection info
            collection_exists = False
            collection_points_count = 0

            try:
                collection_info = await client.get_collection(self._collection_name)
                collection_exists = True
                collection_points_count = collection_info.points_count
            except Exception:
                # Collection might not exist yet, which is fine
                pass

            return {
                "connection": True,
                "collection_exists": collection_exists,
                "points_count": collection_points_count,
                "collection_name": self._collection_name
            }
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return {
                "connection": False,
                "error": str(e)
            }

    async def close_connections(self):
        if self._client:
            await self._client.close()
            logger.info("QdrantDB client connection closed")