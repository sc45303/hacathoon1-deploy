"""
Neon Serverless Postgres connector module for the RAG chatbot backend.

This module provides async methods to interact with Neon Serverless Postgres
for storing book content, metadata, and other structured data.
"""
import os
import asyncio
import asyncpg
from typing import Dict, List, Optional, Any
from uuid import uuid4
from datetime import datetime
import logging

from ..core.config import settings

logger = logging.getLogger(__name__)


class NeonDB:
    """
    Neon Serverless Postgres connector with lazy initialization.

    The connection pool is created on first use, not during initialization,
    to satisfy the requirement of no automatic queries on startup.
    """

    def __init__(self):
        self._pool = None
        self._connection_string = settings.NEON_DATABASE_URL
        self._pool_creation_time = None

    async def _get_pool(self):
        """
        Get the connection pool, creating it if it doesn't exist.
        """
        if self._pool is None:
            if not self._connection_string:
                raise ValueError("NEON_DATABASE_URL environment variable not set")

            # Create the connection pool
            self._pool = await asyncpg.create_pool(
                dsn=self._connection_string,
                min_size=1,
                max_size=10,
                command_timeout=60,
                ssl='require'  # Required for Neon Serverless
            )
            logger.info("NeonDB connection pool created")

        return self._pool

    async def insert_chunk(self, chunk_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Inserts a new book text chunk into the database.

        Args:
            chunk_data: Dictionary containing:
                - book_id: String identifier for the book
                - chapter: String chapter identifier
                - section: String section identifier
                - content: Text content of the chunk
                - source_metadata: JSON object with metadata (optional)

        Returns:
            Dict with the inserted chunk data including auto-generated id and timestamps
        """
        pool = await self._get_pool()

        # Generate a new UUID for the chunk
        chunk_id = str(uuid4())
        created_at = datetime.utcnow()

        query = """
            INSERT INTO book_content_chunks (
                id, chunk_hash, book_id, chapter, section, source_file,
                chunk_index, content, content_length, embedding_status, created_at
            )
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11)
            RETURNING id, chunk_hash, book_id, chapter, section, source_file,
                      chunk_index, content, content_length, embedding_status, created_at
        """

        # Retry logic for robustness against connection issues
        max_retries = 3
        for attempt in range(max_retries):
            try:
                async with pool.acquire() as connection:
                    record = await connection.fetchrow(
                        query,
                        chunk_id,
                        chunk_data.get('chunk_hash'),
                        chunk_data.get('book_id'),
                        chunk_data.get('chapter'),
                        chunk_data.get('section'),
                        chunk_data.get('source_file'),
                        chunk_data.get('chunk_index'),
                        chunk_data.get('content'),
                        len(chunk_data.get('content', '')),
                        'pending',  # embedding_status
                        created_at
                    )

                    result = dict(record)
                    logger.info(f"Successfully inserted chunk with ID: {result['id']}")
                    return result
            except asyncpg.exceptions.InterfaceError as e:
                # This error occurs when connection is closed
                if "connection was closed" in str(e).lower() and attempt < max_retries - 1:
                    logger.warning(f"Connection closed during insert_chunk (attempt {attempt + 1}), retrying...")
                    # Recreate the pool to get fresh connections
                    self._pool = None
                    await self._get_pool()
                    continue
                else:
                    logger.error(f"Failed to insert chunk after {max_retries} attempts: {e}")
                    raise
            except Exception as e:
                logger.error(f"Failed to insert chunk: {e}")
                raise

    async def get_chunks_by_book_id(self, book_id: str) -> List[Dict[str, Any]]:
        """
        Retrieves all chunks associated with a specific book.

        Args:
            book_id: String identifier for the book

        Returns:
            List of chunk dictionaries
        """
        pool = await self._get_pool()

        query = """
            SELECT id, chunk_hash, book_id, chapter, section, source_file,
                   chunk_index, content, content_length, embedding_status, created_at
            FROM book_content_chunks
            WHERE book_id = $1
            ORDER BY created_at
        """

        # Retry logic for robustness against connection issues
        max_retries = 3
        for attempt in range(max_retries):
            try:
                async with pool.acquire() as connection:
                    records = await connection.fetch(query, book_id)
                    result = [dict(record) for record in records]
                    logger.info(f"Retrieved {len(result)} chunks for book ID: {book_id}")
                    return result
            except asyncpg.exceptions.InterfaceError as e:
                # This error occurs when connection is closed
                if "connection was closed" in str(e).lower() and attempt < max_retries - 1:
                    logger.warning(f"Connection closed during get_chunks_by_book_id (attempt {attempt + 1}), retrying...")
                    # Recreate the pool to get fresh connections
                    self._pool = None
                    await self._get_pool()
                    continue
                else:
                    logger.error(f"Failed to get chunks for book ID after {max_retries} attempts: {e}")
                    raise
            except Exception as e:
                logger.error(f"Failed to get chunks for book ID {book_id}: {e}")
                raise

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieves a specific chunk by its UUID.

        Args:
            chunk_id: UUID string of the chunk

        Returns:
            Single chunk dictionary or None if not found
        """
        pool = await self._get_pool()

        query = """
            SELECT id, chunk_hash, book_id, chapter, section, source_file,
                   chunk_index, content, content_length, embedding_status, created_at
            FROM book_content_chunks
            WHERE id = $1
        """

        async with pool.acquire() as connection:
            try:
                record = await connection.fetchrow(query, chunk_id)
                if record:
                    result = dict(record)
                    logger.debug(f"Retrieved chunk with ID: {result['id']}")
                    return result
                else:
                    logger.debug(f"Chunk with ID {chunk_id} not found")
                    return None
            except Exception as e:
                logger.error(f"Failed to get chunk with ID {chunk_id}: {e}")
                raise

    async def get_chunks_by_ids(self, chunk_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieves multiple chunks by their UUIDs.

        Args:
            chunk_ids: List of UUID strings of the chunks

        Returns:
            List of chunk dictionaries
        """
        if not chunk_ids:
            logger.debug("No chunk IDs provided to retrieve")
            return []

        pool = await self._get_pool()

        # Create placeholders for the IN clause
        placeholders = ','.join([f'${i+1}' for i in range(len(chunk_ids))])
        query = f"""
            SELECT id, chunk_hash, book_id, chapter, section, source_file,
                   chunk_index, content, content_length, embedding_status, created_at
            FROM book_content_chunks
            WHERE id IN ({placeholders})
            ORDER BY created_at
        """

        async with pool.acquire() as connection:
            try:
                records = await connection.fetch(query, *chunk_ids)
                result = [dict(record) for record in records]

                # Preserve the original order of chunk_ids as much as possible
                # by creating a mapping and reordering results
                id_to_record = {record['id']: record for record in result}
                ordered_result = []
                for chunk_id in chunk_ids:
                    if chunk_id in id_to_record:
                        ordered_result.append(id_to_record[chunk_id])

                logger.debug(f"Retrieved {len(ordered_result)} chunks by IDs")
                return ordered_result
            except Exception as e:
                logger.error(f"Failed to get chunks by IDs: {e}")
                raise

    async def get_chunks_by_ids_with_dedup(self, chunk_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieves multiple chunks by their UUIDs with chunk_id-level deduplication.

        Args:
            chunk_ids: List of UUID strings of the chunks (may contain duplicates)

        Returns:
            List of unique chunk dictionaries, preserving first occurrence order
        """
        if not chunk_ids:
            logger.debug("No chunk IDs provided to retrieve")
            return []

        # Deduplicate the chunk_ids while preserving order
        unique_chunk_ids = []
        seen = set()
        for chunk_id in chunk_ids:
            if chunk_id not in seen:
                unique_chunk_ids.append(chunk_id)
                seen.add(chunk_id)

        return await self.get_chunks_by_ids(unique_chunk_ids)

    async def update_chunk_content(self, chunk_id: str, new_content: str) -> bool:
        """
        Updates the content of an existing chunk.

        Args:
            chunk_id: UUID string of the chunk to update
            new_content: String with new content

        Returns:
            Boolean indicating success
        """
        pool = await self._get_pool()

        query = """
            UPDATE book_content_chunks
            SET content = $2, content_length = $3, updated_at = $4
            WHERE id = $1
            RETURNING id
        """

        async with pool.acquire() as connection:
            try:
                record = await connection.fetchrow(query, chunk_id, new_content, len(new_content), datetime.utcnow())
                if record:
                    logger.info(f"Successfully updated chunk with ID: {chunk_id}")
                    return True
                else:
                    logger.warning(f"Chunk with ID {chunk_id} not found for update")
                    return False
            except Exception as e:
                logger.error(f"Failed to update chunk with ID {chunk_id}: {e}")
                raise

    async def chunk_exists_by_hash(self, chunk_hash: str) -> bool:
        """
        Checks if a chunk with the given hash already exists in the database.

        Args:
            chunk_hash: SHA-256 hash to check for

        Returns:
            Boolean indicating if the chunk exists
        """
        pool = await self._get_pool()

        query = """
            SELECT COUNT(*)
            FROM book_content_chunks
            WHERE chunk_hash = $1
        """

        # Retry logic for robustness against connection issues
        max_retries = 3
        for attempt in range(max_retries):
            try:
                async with pool.acquire() as connection:
                    result = await connection.fetchval(query, chunk_hash)
                    exists = result > 0
                    logger.info(f"Chunk with hash {chunk_hash} {'exists' if exists else 'does not exist'}")
                    return exists
            except asyncpg.exceptions.InterfaceError as e:
                # This error occurs when connection is closed
                if "connection was closed" in str(e).lower() and attempt < max_retries - 1:
                    logger.warning(f"Connection closed during chunk_exists_by_hash (attempt {attempt + 1}), retrying...")
                    # Recreate the pool to get fresh connections
                    self._pool = None
                    await self._get_pool()
                    continue
                else:
                    logger.error(f"Failed to check if chunk exists by hash after {max_retries} attempts: {e}")
                    raise
            except Exception as e:
                logger.error(f"Failed to check if chunk exists by hash: {e}")
                raise

    async def update_embedding_status(self, chunk_id: str, status: str) -> bool:
        """
        Updates the embedding status of a chunk.

        Args:
            chunk_id: UUID string of the chunk
            status: New status value ('pending', 'processing', 'completed', 'failed')

        Returns:
            Boolean indicating success
        """
        pool = await self._get_pool()

        query = """
            UPDATE book_content_chunks
            SET embedding_status = $2, updated_at = $3
            WHERE id = $1
            RETURNING id
        """

        # Retry logic for robustness against connection issues
        max_retries = 3
        for attempt in range(max_retries):
            try:
                async with pool.acquire() as connection:
                    record = await connection.fetchrow(query, chunk_id, status, datetime.utcnow())
                    if record:
                        logger.info(f"Successfully updated embedding status for chunk with ID: {chunk_id} to {status}")
                        return True
                    else:
                        logger.warning(f"Chunk with ID {chunk_id} not found for status update")
                        return False
            except asyncpg.exceptions.InterfaceError as e:
                # This error occurs when connection is closed
                if "connection was closed" in str(e).lower() and attempt < max_retries - 1:
                    logger.warning(f"Connection closed during update_embedding_status (attempt {attempt + 1}), retrying...")
                    # Recreate the pool to get fresh connections
                    self._pool = None
                    await self._get_pool()
                    continue
                else:
                    logger.error(f"Failed to update embedding status after {max_retries} attempts: {e}")
                    raise
            except Exception as e:
                logger.error(f"Failed to update embedding status for chunk with ID {chunk_id}: {e}")
                raise

    async def delete_chunk(self, chunk_id: str) -> bool:
        """
        Removes a chunk from the database.

        Args:
            chunk_id: UUID string of the chunk to delete

        Returns:
            Boolean indicating success
        """
        pool = await self._get_pool()

        query = """
            DELETE FROM book_content_chunks
            WHERE id = $1
            RETURNING id
        """

        async with pool.acquire() as connection:
            try:
                record = await connection.fetchrow(query, chunk_id)
                if record:
                    logger.info(f"Successfully deleted chunk with ID: {chunk_id}")
                    return True
                else:
                    logger.warning(f"Chunk with ID {chunk_id} not found for deletion")
                    return False
            except Exception as e:
                logger.error(f"Failed to delete chunk with ID {chunk_id}: {e}")
                raise

    async def ensure_tables(self) -> bool:
        """
        Creates required tables if they don't exist.

        Returns:
            Boolean indicating success
        """
        try:
            pool = await self._get_pool()
            async with pool.acquire() as connection:
                # Create book_content_chunks table
                await connection.execute("""
                    CREATE TABLE IF NOT EXISTS book_content_chunks (
                        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                        chunk_hash VARCHAR(64) UNIQUE NOT NULL,
                        book_id VARCHAR(255) NOT NULL,
                        chapter VARCHAR(255),
                        section VARCHAR(255),
                        source_file VARCHAR(500) NOT NULL,
                        chunk_index INTEGER NOT NULL,
                        content TEXT NOT NULL,
                        content_length INTEGER NOT NULL,
                        embedding_status VARCHAR(20) DEFAULT 'pending',
                        created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                        updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
                    );
                """)

                # Create indexes
                await connection.execute("""
                    CREATE INDEX IF NOT EXISTS idx_book_content_chunks_book_id
                    ON book_content_chunks(book_id);
                """)
                await connection.execute("""
                    CREATE INDEX IF NOT EXISTS idx_book_content_chunks_source_file
                    ON book_content_chunks(source_file);
                """)
                await connection.execute("""
                    CREATE INDEX IF NOT EXISTS idx_book_content_chunks_chunk_hash
                    ON book_content_chunks(chunk_hash);
                """)
                await connection.execute("""
                    CREATE INDEX IF NOT EXISTS idx_book_content_chunks_embedding_status
                    ON book_content_chunks(embedding_status);
                """)

                # Create ingestion_runs table
                await connection.execute("""
                    CREATE TABLE IF NOT EXISTS ingestion_runs (
                        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                        run_identifier VARCHAR(255) NOT NULL,
                        start_time TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                        end_time TIMESTAMP WITH TIME ZONE,
                        status VARCHAR(20) DEFAULT 'running',
                        total_files_processed INTEGER DEFAULT 0,
                        total_chunks_created INTEGER DEFAULT 0,
                        files_with_errors INTEGER DEFAULT 0,
                        completion_percentage DECIMAL(5,2) DEFAULT 0.00,
                        notes TEXT
                    );
                """)

                # Create indexes for ingestion_runs
                await connection.execute("""
                    CREATE INDEX IF NOT EXISTS idx_ingestion_runs_start_time
                    ON ingestion_runs(start_time);
                """)
                await connection.execute("""
                    CREATE INDEX IF NOT EXISTS idx_ingestion_runs_status
                    ON ingestion_runs(status);
                """)

                logger.info("Ensured all required tables exist")
                return True
        except Exception as e:
            logger.error(f"Failed to ensure tables exist: {e}")
            return False

    async def validate_connection(self) -> bool:
        """
        Checks if the database connection is working.

        Returns:
            Boolean indicating connection status
        """
        try:
            pool = await self._get_pool()
            async with pool.acquire() as connection:
                # Run a simple query to test the connection
                await connection.fetchval("SELECT 1")
                logger.info("NeonDB connection validation successful")
                return True
        except Exception as e:
            logger.error(f"NeonDB connection validation failed: {e}")
            return False

    async def health_check(self) -> Dict[str, Any]:
        """
        Performs a comprehensive health check of the database connection.

        Returns:
            Dict with health status details
        """
        status = {
            "service": "neon_db",
            "status": "unknown",
            "timestamp": datetime.utcnow().isoformat(),
            "details": {}
        }

        try:
            is_connected = await self.validate_connection()
            if is_connected:
                status["status"] = "healthy"
                status["details"]["connection"] = "OK"
            else:
                status["status"] = "unhealthy"
                status["details"]["connection"] = "Failed"
        except Exception as e:
            status["status"] = "unhealthy"
            status["details"]["error"] = str(e)

        return status

    async def close_connections(self):
        """
        Safely closes all connections in the pool.
        """
        if self._pool:
            try:
                await self._pool.close()
                logger.info("NeonDB connections closed")
            except Exception as e:
                logger.warning(f"Error closing NeonDB connections: {e}")
            finally:
                self._pool = None