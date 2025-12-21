# Quickstart: RAG Query System

## Overview

This guide explains how to implement the RAG (Retrieval Augmented Generation) Query System that allows users to ask natural language questions about book content and receive accurate answers based on the ingested materials.

## Architecture Overview

The system is built in a modular fashion with these key components:

1. **Retriever** (`app/rag/retriever.py`) - Performs vector similarity search in Qdrant
2. **Database Access** (`app/db/neon.py`) - Fetches full chunk text from Neon Postgres
3. **Prompt Engine** (`app/rag/prompt.py`) - Constructs RAG prompts with strict rules
4. **Generator** (`app/rag/generator.py`) - Generates answers using OpenAI
5. **Pipeline** (`app/rag/pipeline.py`) - Orchestrates the end-to-end process
6. **API Layer** (`app/api/routes/query.py`) - Exposes the `/query` endpoint

## Prerequisites

- Python 3.11+
- FastAPI framework
- Async Python environment
- OpenAI SDK
- Access to:
  - Neon Postgres with `book_content_chunks` table
  - Qdrant with vector embeddings
  - OpenAI API key

## Implementation Steps

### 1. Set up the Retriever Module

Create `app/rag/retriever.py` with:
- Async function for Qdrant similarity search
- Top-K retrieval (default K=5) 
- Extraction of chunk_id, chunk_hash, similarity score, and metadata

### 2. Implement Database Access

Update `app/db/neon.py` functions to:
- Fetch full chunk text by chunk_id
- Preserve chunk ordering 
- Implement safe deduplication

### 3. Build the Prompt Engine

Create `app/rag/prompt.py` with:
- Strict RAG prompt template
- Instructions to use ONLY retrieved context
- Hallucination prevention mechanisms
- Fallback message for insufficient context

### 4. Develop Answer Generation

Create `app/rag/generator.py` with:
- OpenAI Chat Completion integration
- Low temperature (≤ 0.3) setting
- Proper system/user message formatting
- Error handling for API failures

### 5. Orchestrate the Pipeline

Update `app/rag/pipeline.py` to:
- Combine all components into a cohesive workflow
- Handle the full RAG process: query → embedding → retrieval → generation
- Return properly formatted answers and sources

### 6. Expose the API

Create `app/api/routes/query.py` with:
- POST `/query` endpoint
- Input validation using Pydantic
- Proper response formatting
- Error handling and logging

## Testing

After implementation, verify:

1. **Functional Tests**:
   - Submit questions with known answers in the book content
   - Verify responses are accurate and grounded in the content
   - Test questions with no relevant content to ensure "Answer not found in provided content" is returned
   - Verify source citations are included in responses

2. **Performance Tests**:
   - Ensure responses are returned within 10 seconds
   - Test concurrent request handling

3. **Error Handling Tests**:
   - Test with unavailable Qdrant service
   - Test with unavailable OpenAI service
   - Test with malformed inputs

## Environment Configuration

The system requires these environment variables:

```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
NEON_DATABASE_URL=your_neon_database_url
QDRANT_COLLECTION_NAME=your_collection_name
EMBEDDING_MODEL_NAME=your_embedding_model_name
```

## Next Steps

After completing the implementation:

1. Run comprehensive tests to validate all functionality
2. Perform performance optimization if needed
3. Update documentation
4. Prepare for deployment to Vercel