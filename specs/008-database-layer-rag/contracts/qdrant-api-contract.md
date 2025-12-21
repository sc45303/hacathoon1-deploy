# Qdrant Vector Database API Contract

## Overview
This document defines the API contracts for interacting with the Qdrant Cloud vector database connector in the RAG chatbot system.

## Connection Management

### Initialization
- Method: `init_client()`
- Description: Initializes the async Qdrant client connection
- Parameters: None
- Returns: Qdrant client object
- Behavior: Establishs initial connection using environment variables; lazy initialization means actual connection happens on first operation

### Close Connections
- Method: `close_connections()`
- Description: Closes the Qdrant client connection
- Parameters: None
- Returns: None
- Behavior: Safely closes the client connection

## Collection Management

### Ensure Collection Exists
- Method: `ensure_collection()`
- Description: Creates the collection if it doesn't exist, with proper vector configuration
- Parameters: None
- Returns: Boolean indicating if collection exists or was created successfully
- Behavior: Creates collection with 1536-dimensional vectors using cosine distance
- Error handling: Raises exception on connection or creation issues

## Vector Operations

### Upload Vector
- Method: `upload_vector(embedding, payload)`
- Description: Uploads a vector embedding to the Qdrant collection
- Parameters:
  - embedding: List of floats representing the vector (typically 1536-dimensional for OpenAI ada-002)
  - payload: Dictionary containing metadata including chunk_id, book_id, chapter, and section
- Returns: Boolean indicating success
- Error handling: Raises exception on invalid vector dimensions or connection issues

### Batch Upload Vectors
- Method: `batch_upload_vectors(vectors_data)`
- Description: Uploads multiple vector embeddings in a single operation
- Parameters:
  - vectors_data: List of tuples (embedding, payload) to upload
- Returns: Boolean indicating success
- Error handling: Raises exception if any vector fails to upload

### Search Similar Vectors
- Method: `search_similar(query_embedding, top_k=5)`
- Description: Performs similarity search to find vectors similar to the query
- Parameters:
  - query_embedding: List of floats representing the query vector
  - top_k: Integer specifying number of similar vectors to return (default: 5)
- Returns: List of matching vectors with their payloads and similarity scores
- Error handling: Returns empty list on connection issues; raises exception on invalid query vector

### Get Vector by ID
- Method: `get_vector_by_id(vector_id)`
- Description: Retrieves a specific vector by its ID
- Parameters:
  - vector_id: String or integer ID of the vector
- Returns: Vector data with payload or None if not found
- Error handling: Returns None if not found; raises exception on connection issues

### Delete Vector
- Method: `delete_vector(vector_id)`
- Description: Removes a vector from the collection
- Parameters:
  - vector_id: String or integer ID of the vector to delete
- Returns: Boolean indicating success
- Error handling: Returns False if vector doesn't exist; raises exception on connection issues

## Payload Operations

### Update Payload
- Method: `update_payload(vector_id, new_payload)`
- Description: Updates the payload (metadata) for a specific vector
- Parameters:
  - vector_id: String or integer ID of the vector
  - new_payload: Dictionary with updated metadata
- Returns: Boolean indicating success
- Error handling: Returns False if vector doesn't exist; raises exception on connection issues

## Validation Utilities

### Validate Connection
- Method: `validate_connection()`
- Description: Checks if the Qdrant connection is working
- Parameters: None
- Returns: Boolean indicating connection status
- Error handling: Returns False on any connection issue

### Health Check
- Method: `health_check()`
- Description: Performs a comprehensive health check of the Qdrant connection
- Parameters: None
- Returns: Dict with health status details
- Error handling: Returns health status even if connection issues exist