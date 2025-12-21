# Neon Postgres API Contract

## Overview
This document defines the API contracts for interacting with the Neon Postgres database connector in the RAG chatbot system.

## Connection Management

### Initialization
- Method: `init_connection_pool()`
- Description: Initializes the async connection pool to Neon Serverless Postgres
- Parameters: None
- Returns: Connection pool object
- Behavior: Establishes initial connection using environment variables; lazy initialization means actual connection happens on first query

### Close Connections
- Method: `close_connections()`
- Description: Closes all active connections in the pool
- Parameters: None
- Returns: None
- Behavior: Safely closes all connections without affecting other operations

## Data Operations

### Insert Book Chunk
- Method: `insert_chunk(chunk_data)`
- Description: Inserts a new book text chunk into the database
- Parameters:
  - chunk_data: Dictionary containing:
    - book_id: String identifier for the book
    - chapter: String chapter identifier
    - section: String section identifier
    - content: Text content of the chunk
    - source_metadata: JSON object with metadata
- Returns: Dict with the inserted chunk data including auto-generated id and timestamps
- Error handling: Raises exception on invalid data or database connection issues

### Get Chunks by Book ID
- Method: `get_chunks_by_book_id(book_id)`
- Description: Retrieves all chunks associated with a specific book
- Parameters:
  - book_id: String identifier for the book
- Returns: List of chunk dictionaries
- Error handling: Returns empty list if no chunks found; raises exception on connection issues

### Get Chunk by ID
- Method: `get_chunk_by_id(chunk_id)`
- Description: Retrieves a specific chunk by its UUID
- Parameters:
  - chunk_id: UUID string of the chunk
- Returns: Single chunk dictionary or None if not found
- Error handling: Returns None if not found; raises exception on connection issues

### Update Chunk Content
- Method: `update_chunk_content(chunk_id, new_content)`
- Description: Updates the content of an existing chunk
- Parameters:
  - chunk_id: UUID string of the chunk to update
  - new_content: String with new content
- Returns: Boolean indicating success
- Error handling: Returns False if chunk doesn't exist; raises exception on connection issues

### Delete Chunk
- Method: `delete_chunk(chunk_id)`
- Description: Removes a chunk from the database
- Parameters:
  - chunk_id: UUID string of the chunk to delete
- Returns: Boolean indicating success
- Error handling: Returns False if chunk doesn't exist; raises exception on connection issues

## Validation Utilities

### Validate Connection
- Method: `validate_connection()`
- Description: Checks if the database connection is working
- Parameters: None
- Returns: Boolean indicating connection status
- Error handling: Returns False on any connection issue

### Health Check
- Method: `health_check()`
- Description: Performs a comprehensive health check of the database connection
- Parameters: None
- Returns: Dict with health status details
- Error handling: Returns health status even if connection issues exist