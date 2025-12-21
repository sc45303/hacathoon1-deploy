# RAG Chatbot Backend

This is the backend service for the RAG (Retrieval-Augmented Generation) Chatbot system integrated with a Docusaurus-based book. The backend provides API endpoints for chat interactions, health checks, and other services needed by the embedded chatbot UI.

## Project Architecture

The backend follows a modular architecture with clear separation of concerns:

- **`/app/api`**: API layer with route definitions (v1, v2, etc.)
- **`/app/core`**: Core layer with configuration and logging
- **`/app/services`**: Service layer with business logic
- **`/app/db`**: Database layer with connection utilities
- **`/app/models`**: Pydantic models for data validation
- **`/scripts`**: One-time scripts for data ingestion, migrations, etc.

This structure supports the modularity principle from the project constitution, allowing each layer to be developed and tested independently.

## Local Setup

1. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

2. **Create a virtual environment** (recommended):
   ```bash
   python -m venv venv
   ```

3. **Activate the virtual environment**:
   - On Windows:
     ```bash
     venv\Scripts\activate
     ```
   - On macOS/Linux:
     ```bash
     source venv/bin/activate
     ```

4. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

5. **Create environment file**:
   Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

   You can modify values in `.env` if needed, but defaults should work for initial testing.

6. **Start the development server**:
   ```bash
   uvicorn app.main:app --reload
   ```

## Database Layer Setup

The backend includes a database layer that integrates both Neon Serverless Postgres and Qdrant Cloud:

### Neon Serverless Postgres
- Purpose: Stores structured book content, metadata, and relationships
- Schema: Contains `book_chunks` table with fields for ID, book_id, chapter, section, content, source_metadata, and timestamps
- Configuration: Uses connection pooling for efficient resource management

### Qdrant Vector Database
- Purpose: Stores vector embeddings for semantic search and similarity matching
- Schema: Contains `book_chunks_vectors` collection with 1536-dimensional vectors and payloads with references to Postgres records
- Configuration: Uses HTTP protocol with API key authentication

### Environment Configuration
Update your `.env` file with appropriate values for your Neon and Qdrant instances:
```
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname
QDRANT_URL=https://your-cluster.xxxxxx.us-east-1-1.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_chunks_vectors
```

> **Note**: The system is designed with Qdrant Free Tier limits in mind. Operations are optimized to minimize resource usage and avoid exceeding free tier restrictions.

## Ingestion Pipeline Setup

The backend also includes a book ingestion pipeline that reads content from Markdown files and prepares it for RAG usage:

### Ingestion Script Usage
- Location: `scripts/ingest_book.py`
- Purpose: Processes Markdown files from book content and stores chunks in Neon Postgres with corresponding embeddings in Qdrant
- Execution: Run manually via command line (does not run automatically on startup)
- Idempotency: Safe to re-run without duplicating content

### Ingestion Parameters
Configuration for the ingestion pipeline is handled via environment variables:
```
# Ingestion Configuration
BOOK_SOURCE_DIR=./path/to/book/markdown/files
INGESTION_CHUNK_SIZE=400
INGESTION_OVERLAP=50
OPENAI_EMBEDDING_MODEL=text-embedding-ada-002
```

### Running the Ingestion Pipeline
```bash
cd backend
python scripts/ingest_book.py --source-dir "/path/to/book/markdown/files" --book-id "my-book-title"
```

For more information on using the ingestion pipeline, refer to the quickstart guide in the specification.

## Running the Application

Once you've completed the setup:

1. Make sure you're in the `backend` directory
2. Ensure your virtual environment is activated
3. Run: `uvicorn app.main:app --reload`
4. The server will start on `http://localhost:8000` by default

## Health Check Endpoint

The backend provides a health check endpoint to verify that the service is operational:

- **Endpoint**: `GET /health`
- **Response**: JSON with status, message, and timestamp
- **Example**:
  ```json
  {
    "status": "healthy",
    "message": "Backend is operational",
    "timestamp": "2025-12-16T10:30:00"
  }
  ```

## Environment Variables

The application uses several environment variables for configuration. The `.env.example` file contains all required variables with placeholder values. For local development, the default values should work, but you may need to update them for production deployment.

## Future Development

This backend skeleton is designed to support the following future features:

- Integration with Neon Postgres for storing book content and metadata
- Connection to Qdrant vector store for semantic search
- Implementation of RAG logic for chat interactions
- OpenAI integration for response generation

## Contributing

When adding new features to the backend:

1. Follow the modular architecture by placing code in the appropriate layer
2. Use Pydantic models for all request/response validation
3. Add proper logging to all new components
4. Ensure all new endpoints follow RESTful principles
5. Update this README with any new setup or configuration requirements

## API Endpoints

The backend provides several API endpoints for different functionalities:

### Agent Endpoint
- **Endpoint**: `POST /agent/query`
- **Purpose**: Unified endpoint that acts as a central decision-making agent to route queries to appropriate answering strategies
- **Modes**:
  - **Selected Text Mode**: When `selected_text` parameter is provided, answers only from the provided text
  - **Book RAG Mode**: Default mode, uses the existing RAG pipeline to retrieve relevant information from book content
  - **General Knowledge Mode**: When `mode` is explicitly set to "general", uses direct LLM completion without retrieval
- **Request Format**:
  ```json
  {
    "question": "string (required)",
    "selected_text": "string (optional)",
    "mode": "selected_text | book | general (optional)"
  }
  ```
- **Response Format**:
  ```json
  {
    "answer": "string",
    "sources": [],
    "mode": "selected_text | book | general"
  }
  ```

### Query Endpoint
- **Endpoint**: `POST /query`
- **Purpose**: Direct RAG query endpoint that retrieves information from book content
- **Request Format**:
  ```json
  {
    "question": "string"
  }
  ```
- **Response Format**:
  ```json
  {
    "answer": "string",
    "sources": []
  }
  ```

### Health Check Endpoint
- **Endpoint**: `GET /health`
- **Purpose**: Verify backend service is operational

## Next Steps

After verifying the backend skeleton is working:

1. Implement database connectors in the `/app/db` directory
2. Add data models in the `/app/models` directory
3. Create service layer functionality in `/app/services`
4. Add API endpoints in `/app/api/v1/`
5. Implement the RAG logic in the service layer