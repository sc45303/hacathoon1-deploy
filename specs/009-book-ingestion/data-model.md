# Data Model: Book Ingestion Pipeline for RAG Chatbot

## Overview
This document defines the data models for the book ingestion pipeline, specifying how book content is structured and stored in both the structured database (Neon Postgres) and the vector database (Qdrant) for the RAG chatbot system.

## Structured Database Schema (Neon Postgres)

### Table: book_content_chunks
Stores the text chunks of books along with their metadata.

```sql
CREATE TABLE book_content_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chunk_hash VARCHAR(64) UNIQUE NOT NULL, -- SHA-256 hash for idempotency
    book_id VARCHAR(255) NOT NULL,
    chapter VARCHAR(255),
    section VARCHAR(255),
    source_file VARCHAR(500) NOT NULL, -- Relative path from book root
    chunk_index INTEGER NOT NULL, -- Sequential number of chunk in source file
    content TEXT NOT NULL,
    content_length INTEGER NOT NULL, -- Character count of content
    embedding_status VARCHAR(20) DEFAULT 'pending', -- pending, processing, completed, failed
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for efficient querying
CREATE INDEX idx_book_content_chunks_book_id ON book_content_chunks(book_id);
CREATE INDEX idx_book_content_chunks_source_file ON book_content_chunks(source_file);
CREATE INDEX idx_book_content_chunks_chunk_hash ON book_content_chunks(chunk_hash);
CREATE INDEX idx_book_content_chunks_embedding_status ON book_content_chunks(embedding_status);
CREATE INDEX idx_book_content_chunks_created_at ON book_content_chunks(created_at);
```

#### Field Definitions
- `id`: Unique identifier for each chunk (UUID, primary key)
- `chunk_hash`: SHA-256 hash of the content and source location for idempotency
- `book_id`: Identifier for the book this chunk belongs to
- `chapter`: Chapter name or number where the chunk appears
- `section`: Section within the chapter
- `source_file`: Relative path from the book root directory to the source file
- `chunk_index`: Sequential number of this chunk within the source file
- `content`: The actual text content of the chunk
- `content_length`: Character count of the content for statistics
- `embedding_status`: Status of the embedding generation process (pending, processing, completed, failed)
- `created_at`: Timestamp when the chunk was added to the database
- `updated_at`: Timestamp when the chunk was last updated

## Vector Database Schema (Qdrant)

### Collection: book_content_vectors
Stores vector embeddings of book content with references to corresponding structured database records.

#### Vector Configuration
- Size: 1536 dimensions (to match OpenAI ada-002 embeddings)
- Distance: Cosine similarity

#### Payload Fields
- `chunk_id`: UUID referencing the corresponding record in book_content_chunks table
- `chunk_hash`: SHA-256 hash for consistency with Neon database
- `book_id`: Identifier for the book
- `chapter`: Chapter name or number
- `section`: Section within the chapter
- `source_file`: Source file path for reference
- `chunk_index`: Sequential number of this chunk within the source file

#### Example Payload Structure
```json
{
  "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
  "chunk_hash": "a1b2c3d4e5f6...",
  "book_id": "effective-python",
  "chapter": "Chapter 1",
  "section": "Introduction",
  "source_file": "docs/chapter-1/introduction.md",
  "chunk_index": 1
}
```

## Ingestion Process Data Model

### Table: ingestion_runs
Tracks ingestion processes and their outcomes.

```sql
CREATE TABLE ingestion_runs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    run_identifier VARCHAR(255) NOT NULL, -- e.g., "effective-python-2025-12-17"
    start_time TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    end_time TIMESTAMP WITH TIME ZONE,
    status VARCHAR(20) DEFAULT 'running', -- running, completed, failed, cancelled
    total_files_processed INTEGER DEFAULT 0,
    total_chunks_created INTEGER DEFAULT 0,
    files_with_errors INTEGER DEFAULT 0,
    completion_percentage DECIMAL(5,2) DEFAULT 0.00,
    notes TEXT
);

-- Index for efficient querying of recent runs
CREATE INDEX idx_ingestion_runs_start_time ON ingestion_runs(start_time);
CREATE INDEX idx_ingestion_runs_status ON ingestion_runs(status);
```

#### Field Definitions
- `id`: Unique identifier for each ingestion run (UUID, primary key)
- `run_identifier`: Human-readable identifier for the ingestion run
- `start_time`: Timestamp when the ingestion process started
- `end_time`: Timestamp when the ingestion process completed or failed
- `status`: Current status of the ingestion run
- `total_files_processed`: Number of source files processed in this run
- `total_chunks_created`: Number of chunks created in this run
- `files_with_errors`: Number of files that encountered errors during processing
- `completion_percentage`: Percentage of completion (0.00 to 100.00)
- `notes`: Additional information about the ingestion run

## Relationships
There is a one-to-one relationship between records in the book_content_chunks table and vectors in the book_content_vectors collection. The chunk_id in the Qdrant payload references the id in the book_content_chunks table.

## Constraints and Validation Rules

### For book_content_chunks:
- Content length should be between 100 and 2000 characters (enforced at application level)
- chunk_hash must be unique to ensure idempotency
- embedding_status must be one of the allowed values
- book_id should follow alphanumeric with hyphens/underscores pattern

### For book_content_vectors:
- All vector embeddings must be 1536-dimensional
- Payload must contain valid reference to the corresponding Neon record
- chunk_hash in payload must match the one in Neon for consistency

### For ingestion_runs:
- Each run must have a unique run_identifier
- Status values are constrained to valid states
- Completion percentage must be between 0.00 and 100.00

## Indexing Strategy
- Primary keys are indexed by default
- Frequently queried fields (book_id, source_file, chunk_hash, status) have dedicated indexes
- Composite indexes where appropriate for common query patterns

## Data Lifecycle Considerations
- Old ingestion run records could be archived after a period (not implemented in this phase)
- Chunk records are retained as long as they're needed for RAG operations
- The system is designed to allow for future deletion/maintenance operations if needed