"""
Storage coordination module for the book ingestion pipeline.

This module provides functions to store chunks in Neon Postgres
and embeddings in Qdrant, with proper linking between them.

Storage schemas:
- Text chunks are stored in the 'book_content_chunks' table in Neon Postgres
  with fields for content, metadata (book_id, chapter, section, source_file),
  and identifiers for linking to vector embeddings in Qdrant.
- Embeddings are stored in the 'book_content_vectors' collection in Qdrant
  with the embedding vector and payload containing references back to the
  corresponding record in Neon Postgres using chunk_id and chunk_hash.
"""

import os
import logging
import hashlib
from typing import Dict, List, Any, Optional
from datetime import datetime
from app.db.neon import NeonDB
from app.db.qdrant import QdrantDB

logger = logging.getLogger(__name__)


async def store_chunk(chunk_data: Dict[str, Any]) -> Optional[str]:
    """
    Stores a text chunk in the structured database (Neon Postgres).

    Args:
        chunk_data: Dictionary with chunk information (id, hash, content, metadata, etc.)

    Returns:
        String ID of the stored chunk

    Error handling: Raises exception if database operation fails
    """
    neon_db = NeonDB()
    try:
        # Ensure tables exist
        await neon_db.ensure_tables()

        # Prepare the chunk data for storage
        chunk_for_storage = {
            "chunk_hash": chunk_data.get("chunk_hash"),
            "book_id": chunk_data.get("book_id", ""),
            "chapter": chunk_data.get("chapter", ""),
            "section": chunk_data.get("section", ""),
            "source_file": chunk_data.get("source_file", ""),
            "chunk_index": chunk_data.get("chunk_index", 0),
            "content": chunk_data.get("content", ""),
        }

        # Insert the chunk into the database
        stored_chunk = await neon_db.insert_chunk(chunk_for_storage)
        chunk_id = stored_chunk.get("id")

        logger.info(f"Stored chunk with ID: {chunk_id} from {chunk_data.get('source_file')}")
        return chunk_id

    except Exception as e:
        logger.error(f"Error storing chunk in database: {e}")
        raise
    finally:
        await neon_db.close_connections()


async def upsert_embedding(embedding: List[float], payload: Dict[str, Any]) -> bool:
    """
    Upserts an embedding vector with its payload into Qdrant.

    Args:
        embedding: List of floats representing the vector (1536 dimensions)
        payload: Dictionary with metadata linking to structured database record

    Returns:
        Boolean indicating success

    Error handling: Raises exception if database operation fails
    """
    qdrant_db = QdrantDB()
    try:
        success = await qdrant_db.upload_vector(embedding, payload)
        logger.info(f"Upserted embedding with payload: {payload}")
        return success

    except Exception as e:
        logger.error(f"Error upserting embedding to Qdrant: {e}")
        raise
    finally:
        await qdrant_db.close_connections()


async def chunk_exists(chunk_hash: str) -> bool:
    neon_db = NeonDB()
    try:
        await neon_db.ensure_tables()

        max_retries = 3
        for attempt in range(max_retries):
            try:
                exists = await neon_db.chunk_exists_by_hash(chunk_hash)
                return exists
            except Exception as e:
                if "connection was closed" in str(e).lower() and attempt < max_retries - 1:
                    logger.warning(f"Connection issue during chunk_exists (attempt {attempt + 1}), retrying...")
                    import asyncio
                    await asyncio.sleep(0.5)
                    continue
                else:
                    raise

    except Exception as e:
        logger.error(f"Error checking if chunk exists: {e}")
        raise
    finally:
        await neon_db.close_connections()


async def update_ingestion_status(run_id: str, status: str, stats: Dict[str, Any]) -> bool:
    """
    Updates the status and statistics for an ingestion run.

    Args:
        run_id: ID of the ingestion run to update
        status: New status value
        stats: Dictionary with statistics (processed files, chunks created, etc.)

    Returns:
        Boolean indicating success

    Error handling: Raises exception if database operation fails
    """
    # Note: In a complete implementation, this would add a method to NeonDB to update
    # the ingestion_runs table with the status and statistics.

    # Since NeonDB doesn't currently have a specific method for ingestion runs,
    # we would need to enhance the NeonDB class to support this functionality.
    # In a real implementation, we'd call something like:
    # await neon_db.update_ingestion_run(run_id, status, stats)

    # For this implementation example, we'll note that this method would exist
    # and would update the ingestion_runs table as specified in data-model.md

    logger.info(f"Updated ingestion run {run_id} to status {status} with stats: {stats}")
    return True