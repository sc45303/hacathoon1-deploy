# Ingestion Component Interface Contracts

## Overview
This document defines the interface contracts for the book ingestion pipeline components. These contracts specify the methods and data structures for the different modules that make up the ingestion pipeline.

## Markdown Reader Interface Contract

### Module: `app/ingestion/reader.py`

#### `read_markdown_files(source_dir: str) -> List[Dict[str, Any]]`
- **Purpose**: Reads all markdown files from a specified directory and its subdirectories
- **Input**: 
  - `source_dir`: String path to the directory containing markdown files
- **Output**: List of dictionaries containing:
  - `file_path`: Relative path of the markdown file from the source directory
  - `content`: Raw content of the markdown file
  - `metadata`: Dictionary with file metadata (size, modification date, etc.)
- **Error handling**: Raises exception if directory doesn't exist or is inaccessible

#### `extract_text_from_markdown(markdown_content: str) -> str`
- **Purpose**: Strips markdown syntax and extracts plain text content
- **Input**: 
  - `markdown_content`: Raw markdown content as string
- **Output**: Plain text content with markdown syntax removed
- **Error handling**: Returns original content if parsing fails

## Text Chunker Interface Contract

### Module: `app/ingestion/chunker.py`

#### `chunk_text(text: str, file_path: str, chunk_size: int = 400, overlap: int = 50) -> List[Dict[str, Any]]`
- **Purpose**: Splits a text into chunks of specified size with overlap
- **Input**:
  - `text`: Text to be chunked
  - `file_path`: Path of the source file (for metadata)
  - `chunk_size`: Target number of tokens per chunk (default: 400)
  - `overlap`: Number of tokens to overlap between chunks (default: 50)
- **Output**: List of dictionaries containing:
  - `chunk_id`: Generated UUID for the chunk
  - `chunk_hash`: SHA-256 hash of content+source for idempotency
  - `content`: Text content of the chunk
  - `source_file`: Path of the source file
  - `chunk_index`: Sequential index of this chunk in the source file
  - `token_count`: Number of tokens in the chunk
- **Error handling**: Raises exception if text processing fails

#### `calculate_tokens(text: str) -> int`
- **Purpose**: Calculates the number of tokens in a text string
- **Input**: 
  - `text`: Text to count tokens in
- **Output**: Integer number of tokens
- **Error handling**: Returns 0 if calculation fails

## Embedder Interface Contract

### Module: `app/ingestion/embedder.py`

#### `generate_embedding(text: str) -> List[float]`
- **Purpose**: Generates an embedding vector for a text chunk
- **Input**:
  - `text`: Text to generate embedding for
- **Output**: List of floats representing the embedding vector (1536 dimensions)
- **Error handling**: Raises exception if API call fails, includes retry logic

#### `batch_generate_embeddings(texts: List[str]) -> List[List[float]]`
- **Purpose**: Generates embeddings for multiple texts in a batch
- **Input**:
  - `texts`: List of texts to generate embeddings for
- **Output**: List of embedding vectors (each a list of floats)
- **Error handling**: Raises exception if API call fails, includes retry logic

## Storage Interface Contract

### Module: `app/ingestion/storage.py`

#### `store_chunk(chunk_data: Dict[str, Any]) -> str`
- **Purpose**: Stores a text chunk in the structured database (Neon Postgres)
- **Input**:
  - `chunk_data`: Dictionary with chunk information (id, hash, content, metadata, etc.)
- **Output**: String ID of the stored chunk
- **Error handling**: Raises exception if database operation fails

#### `upsert_embedding(embedding: List[float], payload: Dict[str, Any]) -> bool`
- **Purpose**: Upserts an embedding vector with its payload into Qdrant
- **Input**:
  - `embedding`: List of floats representing the vector (1536 dimensions)
  - `payload`: Dictionary with metadata linking to structured database record
- **Output**: Boolean indicating success
- **Error handling**: Raises exception if database operation fails

#### `chunk_exists(chunk_hash: str) -> bool`
- **Purpose**: Checks if a chunk with the given hash already exists in the database
- **Input**:
  - `chunk_hash`: SHA-256 hash to check for
- **Output**: Boolean indicating if the chunk exists
- **Error handling**: Raises exception if database operation fails

#### `update_ingestion_status(run_id: str, status: str, stats: Dict[str, Any]) -> bool`
- **Purpose**: Updates the status and statistics for an ingestion run
- **Input**:
  - `run_id`: ID of the ingestion run to update
  - `status`: New status value
  - `stats`: Dictionary with statistics (processed files, chunks created, etc.)
- **Output**: Boolean indicating success
- **Error handling**: Raises exception if database operation fails

## Ingestion Script Contract

### Module: `scripts/ingest_book.py`

#### Command Line Interface
- `--source-dir`: Required. Path to the directory containing markdown files
- `--book-id`: Required. Identifier for the book being ingested
- `--chunk-size`: Optional. Number of tokens per chunk (default: 400)
- `--overlap`: Optional. Number of overlapping tokens (default: 50)
- `--run-identifier`: Optional. Custom identifier for this ingestion run

#### Main function behavior
- Validates input parameters
- Initializes all required components
- Processes markdown files in batches
- Tracks and reports progress
- Updates ingestion run records
- Handles errors gracefully