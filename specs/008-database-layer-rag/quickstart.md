# Quickstart Guide: Database Layer for RAG Chatbot (Neon + Qdrant)

## Overview
This guide provides instructions for setting up and using the database layer that integrates Neon Serverless Postgres and Qdrant Cloud for the RAG chatbot backend.

## Prerequisites
- Python 3.11+
- Access to Neon Serverless Postgres instance
- Access to Qdrant Cloud (Free Tier) instance
- Environment variables configured as specified below

## Environment Variables Setup
Create a `.env` file in the backend root directory with the following variables:

```bash
# Neon Serverless Postgres
NEON_DATABASE_URL="postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require"

# Qdrant Cloud
QDRANT_URL="https://your-cluster.xxxxxx.us-east-1-1.aws.cloud.qdrant.io:6333"
QDRANT_API_KEY="your-qdrant-api-key"
QDRANT_COLLECTION_NAME="book_chunks_vectors"
```

## Installation
Install the required dependencies for the database layer:

```bash
cd backend
pip install asyncpg qdrant-client python-dotenv pydantic fastapi
```

## Initializing Database Connectors
The database layer provides two independent connectors that can be initialized separately:

### Neon Postgres Connector
```python
from app.db.neon import NeonDB

# Initialize the Neon connector (lazy initialization)
neon_db = NeonDB()

# The actual connection is established on first use
# No automatic queries are executed on initialization
```

### Qdrant Connector
```python
from app.db.qdrant import QdrantDB

# Initialize the Qdrant connector (lazy initialization)
qdrant_db = QdrantDB()

# The actual connection is established on first use
# No automatic queries are executed on initialization
```

## Using the Database Connectors

### Neon Postgres Operations
```python
import asyncio
from app.db.neon import NeonDB

async def example_usage():
    neon_db = NeonDB()
    
    # Insert a new book chunk
    chunk_data = {
        "book_id": "effective-python",
        "chapter": "Chapter 1",
        "section": "Introduction",
        "content": "This is an example book content chunk...",
        "source_metadata": {"page": 1, "author": "John Doe"}
    }
    
    inserted_chunk = await neon_db.insert_chunk(chunk_data)
    print(f"Inserted chunk with ID: {inserted_chunk['id']}")
    
    # Query book chunks by book_id
    chunks = await neon_db.get_chunks_by_book_id("effective-python")
    print(f"Retrieved {len(chunks)} chunks")
    
    # Close connections when done
    await neon_db.close_connections()
```

### Qdrant Operations
```python
import asyncio
from app.db.qdrant import QdrantDB

async def example_usage():
    qdrant_db = QdrantDB()
    
    # Upload a vector embedding with payload
    vector_data = [0.1, 0.2, 0.3, ...]  # 1536-dimensional vector
    payload = {
        "chunk_id": "uuid-of-corresponding-postgres-record",
        "book_id": "effective-python",
        "chapter": "Chapter 1",
        "section": "Introduction"
    }
    
    await qdrant_db.upload_vector(vector_data, payload)
    
    # Perform similarity search
    query_vector = [0.15, 0.25, 0.35, ...]  # Same dimensions
    results = await qdrant_db.search_similar(query_vector, top_k=5)
    
    print(f"Found {len(results)} similar chunks")
    
    # Close connections when done
    await qdrant_db.close_connections()
```

## Connection Validation
To validate that both database connections are properly configured:

```python
from app.db.utils import validate_connections

async def check_connections():
    success = await validate_connections()
    if success:
        print("Both Neon and Qdrant connections are valid")
    else:
        print("One or more connections failed validation")
        
# Run the validation
asyncio.run(check_connections())
```

## Integration with Backend Application
To integrate the database layer with the main FastAPI application:

```python
# In main.py
from fastapi import FastAPI
from app.db.neon import NeonDB
from app.db.qdrant import QdrantDB

app = FastAPI()

@app.on_event("startup")
async def startup_event():
    # Initialize database connections on startup but don't execute queries
    app.state.neon_db = NeonDB()
    app.state.qdrant_db = QdrantDB()
    
    # Validate connections on startup (optional)
    from app.db.utils import validate_connections
    # await validate_connections()  # Uncomment if you want to validate on startup

@app.on_event("shutdown")
async def shutdown_event():
    # Cleanly close database connections
    await app.state.neon_db.close_connections()
    await app.state.qdrant_db.close_connections()
```

## Important Notes
- Database connections are lazily initialized (on first use)
- No queries are executed automatically during application startup
- Both Neon and Qdrant connections respect async/await patterns
- The system is designed with Qdrant Free Tier limits in mind
- Always use try/catch blocks when performing database operations