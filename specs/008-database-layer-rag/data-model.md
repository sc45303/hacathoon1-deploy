# Data Model: Database Layer for RAG Chatbot (Neon + Qdrant)

## Overview
This document defines the data models for the RAG chatbot backend, specifying the schemas for both Neon Postgres (relational data) and Qdrant Cloud (vector embeddings).

## Neon Postgres Schema

### Table: book_chunks
Stores the text chunks of books along with their metadata.

```sql
CREATE TABLE book_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    book_id VARCHAR(255) NOT NULL,
    chapter VARCHAR(255),
    section VARCHAR(255),
    content TEXT NOT NULL,
    source_metadata JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for efficient querying
CREATE INDEX idx_book_chunks_book_id ON book_chunks(book_id);
CREATE INDEX idx_book_chunks_chapter ON book_chunks(chapter);
CREATE INDEX idx_book_chunks_created_at ON book_chunks(created_at);
```

#### Field Definitions
- `id`: Unique identifier for each chunk (UUID, primary key)
- `book_id`: Identifier for the book this chunk belongs to
- `chapter`: Chapter name or number where the chunk appears
- `section`: Section within the chapter
- `content`: The actual text content of the chunk
- `source_metadata`: JSON field for additional metadata (page numbers, authors, etc.)
- `created_at`: Timestamp when the chunk was added to the database

## Qdrant Vector Schema

### Collection: book_chunks_vectors
Stores vector embeddings of book content with references to Postgres records.

#### Vector Configuration
- Size: 1536 dimensions (to match OpenAI ada-002 embeddings)
- Distance: Cosine similarity

#### Payload Fields
- `chunk_id`: UUID referencing the corresponding record in book_chunks table
- `book_id`: Identifier for the book
- `chapter`: Chapter name or number
- `section`: Section within the chapter

#### Example Payload Structure
```json
{
  "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
  "book_id": "effective-python",
  "chapter": "Chapter 1",
  "section": "Introduction"
}
```

## Relationships
There is a one-to-one relationship between records in the book_chunks table and vectors in the book_chunks_vectors collection. The chunk_id in the Qdrant payload references the id in the book_chunks table.

## Constraints and Validations

### Neon Postgres
- Content length validation: minimum 10 characters
- Book ID format validation: alphanumeric with hyphens/underscores
- Chapter and section fields are optional but if present, must follow naming conventions

### Qdrant
- Vector dimension integrity: all vectors must have the expected dimensions
- Payload validation: all required fields must be present
- Reference integrity: chunk_id must correspond to an existing record in Neon