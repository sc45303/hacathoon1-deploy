# Quickstart Guide: Book Ingestion Pipeline for RAG Chatbot

## Overview
This guide provides instructions for setting up and using the book ingestion pipeline that converts book markdown content into structured data and vector embeddings for the RAG chatbot system.

## Prerequisites
- Python 3.11+
- Access to Neon Serverless Postgres instance
- Access to Qdrant Cloud (Free Tier) instance
- OpenAI API key
- Book content in Markdown format (e.g., from Docusaurus project)

## Environment Variables Setup
Create or update your `.env` file in the backend root directory with the following variables:

```bash
# Database Configuration (Neon Postgres)
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname

# Vector Database Configuration (Qdrant)
QDRANT_URL=https://your-cluster.xxxxxx.us-east-1-1.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_content_vectors

# AI Provider Configuration (OpenAI)
OPENAI_API_KEY=your-openai-api-key

# Ingestion Configuration
INGESTION_CHUNK_SIZE=400
INGESTION_OVERLAP=50
BOOK_SOURCE_DIR=./path/to/book/markdown/files
```

## Installation
Install the required dependencies for the ingestion pipeline:

```bash
cd backend
pip install -r requirements.txt
```

## Running the Ingestion Pipeline

### Basic Ingestion
To run the ingestion script with default parameters:

```bash
cd backend
python scripts/ingest_book.py --source-dir "/path/to/book/markdown/files" --book-id "my-book-title"
```

### Advanced Ingestion with Custom Parameters
To run the ingestion script with custom chunk size and overlap:

```bash
cd backend
python scripts/ingest_book.py \
  --source-dir "/path/to/book/markdown/files" \
  --book-id "my-book-title" \
  --chunk-size 600 \
  --overlap 100 \
  --run-identifier "my-book-title-2025-dec"
```

### Understanding the Process
The ingestion pipeline follows these steps:

1. **Read**: Scans the source directory for all `.md` files recursively
2. **Parse**: Converts markdown syntax to plain text
3. **Chunk**: Splits text into configurable-size chunks with overlap
4. **Hash**: Creates unique identifiers for idempotency
5. **Embed**: Generates embeddings using OpenAI API
6. **Store**: Saves content to Neon Postgres and embeddings to Qdrant
7. **Link**: Maintains connection between both storage systems

### Progress Monitoring
The script outputs detailed progress information:

- Files processed count
- Chunks created count
- Current file being processed
- Error notifications (if any)
- Completion percentage

## Verification

### Check Structured Database (Neon Postgres)
Connect to your Neon database and verify the chunks were stored:

```sql
SELECT 
    id, 
    book_id, 
    chapter, 
    section, 
    content_length, 
    created_at 
FROM book_content_chunks 
WHERE book_id = 'my-book-title' 
LIMIT 5;
```

### Check Vector Database (Qdrant)
Using Qdrant dashboard or API, verify that embeddings were created:

```bash
# Get collection info
curl -X GET "{QDRANT_URL}/collections/book_content_vectors" \
  -H "Content-Type: application/json" \
  -H "Api-Key: {YOUR_API_KEY}"
```

## Configuration Options

### Chunk Size
- Default: 400 tokens per chunk
- Larger chunks: Better context, higher embedding costs
- Smaller chunks: More precise retrieval, more chunks to process

### Overlap
- Default: 50 tokens overlap between chunks
- Provides context continuity at chunk boundaries
- Higher overlap: More redundancy, better context recovery

### Book ID
- Identifier for the book being ingested
- Used for querying and organizing content in both databases

## Idempotency
The script can be safely re-run without duplicating content thanks to hash-based chunk identification. On re-run:
- Existing chunks will be detected and skipped
- Only new or modified content will be processed
- No duplicate entries will be created

## Error Handling
- Network errors during API calls will be retried with exponential backoff
- Files that fail to process will be logged but won't stop the entire process
- Progress is tracked so partial failures can be resumed later

## Performance Considerations
- Respect Qdrant Free Tier limits when processing large books
- OpenAI API usage will incur costs based on tokens processed
- Large books may take significant time to process completely
- Monitor API usage to stay within limits

## Integration with Backend
The ingestion pipeline operates independently of the backend service:
- No automatic execution on backend startup
- Backend can run without ingestion components
- Ingested data is immediately available to the RAG system once complete