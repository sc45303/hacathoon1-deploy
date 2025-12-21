# Quickstart Guide: Unified Agentic RAG Chatbot System

## Overview
This guide will help you quickly set up and start using the agent router that classifies and routes queries to appropriate answering strategies.

## Prerequisites
- Python 3.11+
- Existing backend service with RAG pipeline set up
- OpenAI API key
- Qdrant Cloud access
- Neon Postgres connection

## Setup Steps

### 1. Install Dependencies
```bash
# Navigate to the backend directory
cd backend/

# Install required packages (if not already installed)
pip install fastapi pydantic openai qdrant-client psycopg2-binary
```

### 2. Environment Configuration
Ensure the following environment variables are set:
```bash
export OPENAI_API_KEY="your-openai-api-key"
export QDRANT_URL="your-qdrant-cloud-url"
export QDRANT_API_KEY="your-qdrant-api-key"
export NEON_DATABASE_URL="your-neon-database-url"
```

### 3. Run the Service
```bash
# From the backend directory
uvicorn app.api.main:app --reload --port 8000
```

## Usage Examples

### 1. Book RAG Query (Default Mode)
```bash
curl -X POST "http://localhost:8000/agent/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the main themes in this book?"
  }'
```

### 2. Selected Text Query
```bash
curl -X POST "http://localhost:8000/agent/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Can you explain this concept?",
    "selected_text": "The main themes of this book include the struggle between good and evil, the power of friendship, and the importance of perseverance."
  }'
```

### 3. General Knowledge Query
```bash
curl -X POST "http://localhost:8000/agent/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the capital of France?",
    "mode": "general"
  }'
```

## Response Format
All responses follow the same format:
```json
{
  "answer": "string",
  "sources": [],
  "mode": "selected_text | book | general"
}
```

## Testing
Run the tests to verify the agent functionality:
```bash
cd backend/
pytest tests/api/test_agent.py
```