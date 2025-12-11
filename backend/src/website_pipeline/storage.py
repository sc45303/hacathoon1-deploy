"""
Storage Module

This module handles the storage of embeddings in Qdrant vector database.
"""
import os
import time
import uuid
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Load environment variables and initialize Qdrant client
load_dotenv()

qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10
)


def create_collection(collection_name: str = "reg-embedding") -> bool:
    """
    Create a Qdrant collection with appropriate parameters
    
    Args:
        collection_name: Name of the collection to create
        
    Returns:
        True if successful, False otherwise
    """
    try:
        # Check if collection already exists
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]
        
        if collection_name in collection_names:
            # Collection already exists, we can proceed
            return True
        
        # Create a new collection
        # Cohere's embed-english-v3.0 returns 1024-dimensional vectors
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )
        
        return True
    except Exception as e:
        print(f"Could not create collection {collection_name}: {str(e)}")
        return False


def save_chunk_to_qdrant(text_chunk: str, embedding: List[float], metadata: Dict[str, Any], 
                        collection_name: str = "reg-embedding") -> bool:
    """
    Store the text chunk and its embedding in Qdrant with metadata
    
    Args:
        text_chunk: The text content to store
        embedding: The embedding vector
        metadata: Additional metadata to store with the text
        collection_name: Name of the Qdrant collection
        
    Returns:
        True if successful, False otherwise
    """
    try:
        # Generate a unique ID for this record
        record_id = str(uuid.uuid4())
        
        # Prepare the record
        records = [
            models.PointStruct(
                id=record_id,
                vector=embedding,
                payload={
                    "text": text_chunk,
                    "url": metadata.get("url", ""),
                    "position": metadata.get("position", 0),
                    "source": metadata.get("source", ""),
                    "timestamp": time.time()
                }
            )
        ]
        
        # Upload to Qdrant
        qdrant_client.upsert(
            collection_name=collection_name,
            points=records
        )
        
        return True
    except Exception as e:
        print(f"Could not save chunk to Qdrant: {str(e)}")
        return False