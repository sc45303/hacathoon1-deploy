---
id: "001"
title: "Plan Database Layer"
stage: "plan"
date: "2025-12-17"
surface: "agent"
model: "gpt-4"
feature: "008-database-layer-rag"
branch: "008-database-layer-rag"
user: "user"
command: "/sp.plan"
labels: ["database", "neon", "qdrant", "backend", "rag"]
spec: null
ticket: null
adr: null
pr: null
files:
  - "specs/008-database-layer-rag/plan.md"
  - "specs/008-database-layer-rag/research.md"
  - "specs/008-database-layer-rag/data-model.md"
  - "specs/008-database-layer-rag/quickstart.md"
  - "specs/008-database-layer-rag/contracts/neon-api-contract.md"
  - "specs/008-database-layer-rag/contracts/qdrant-api-contract.md"
tests: []
outcome: "Successfully planned the database layer integration for the RAG chatbot backend using Neon Serverless Postgres and Qdrant Cloud. Created comprehensive documentation including technical context, constitution compliance check, project structure, API contracts, and quickstart guide."
evaluation: "All required planning artifacts have been created and the implementation approach aligns with project constitution and requirements."
---

## Prompt

```
Goal:
Plan the database layer integration for the RAG chatbot backend using
Neon Serverless Postgres and Qdrant Cloud, without implementing ingestion
or retrieval logic.

Architecture sketch:
- The database layer will be isolated under /backend/app/db
- Two independent connectors will be created:
  1. neon.py → Neon Serverless Postgres
  2. qdrant.py → Qdrant Cloud vector database
- No database queries or searches will run automatically on app startup
- All database usage will be invoked explicitly by future services

Data model design:
Neon Postgres (relational storage):
- Table: book_chunks
  - id (UUID, primary key)
  - book_id (string)
  - chapter (string)
  - section (string)
  - content (text)
  - source_metadata (JSON)
  - created_at (timestamp)

Qdrant (vector storage):
- Collection: book_chunks_vectors
- Vector:
  - embedding (float array)
- Payload:
  - chunk_id (UUID)
  - book_id
  - chapter
  - section

Environment configuration:
- Neon:
  - NEON_DATABASE_URL
- Qdrant:
  - QDRANT_URL
  - QDRANT_API_KEY
  - QDRANT_COLLECTION_NAME

Design / implementation approach:
- Use async-compatible Postgres client
- Keep database clients lightweight and reusable
- Initialize connections lazily (on first use)
- Centralize all configuration in core/config.py
- Add connection test utilities without executing queries

Important decisions and tradeoffs:
1. Separate Postgres and vector DB:
   - Pros: clean RAG separation, scalable
   - Cons: requires ID synchronization
2. No auto-migrations in this spec:
   - Pros: simpler bootstrap
   - Cons: manual table creation required initially
3. No database access on startup:
   - Pros: faster boot, safer deployment
   - Cons: errors appear only when used

What will NOT be implemented in this plan:
- Book ingestion scripts
- Chunking or preprocessing
- Embedding generation
- Similarity search logic
- RAG pipelines
- API endpoints

Testing / validation strategy:
- Import Neon client without errors
- Import Qdrant client without errors
- Load environment variables successfully
- Start FastAPI app without triggering database calls
- Manually instantiate database clients in a REPL to verify connectivity

Acceptance criteria:
- Database connector modules exist and are importable
- Schema definitions are documented in code comments
- Environment variables are clearly defined
- Backend starts without runtime errors
- Database layer aligns with sp.constitution

Outcome of this plan:
A stable, isolated database layer foundation ready for
future ingestion and RAG retrieval specifications.
```

## Response

```
The implementation planning for the database layer integration has been completed successfully. Here's a summary of what was accomplished:

1. The planning process was initialized using the /sp.plan command, which created the necessary files and structure in the specs/008-database-layer-rag directory.

2. The technical context was defined, specifying:
   - Language: Python 3.11
   - Dependencies: FastAPI, asyncpg, qdrant-client, Pydantic
   - Storage: Neon Serverless Postgres and Qdrant Cloud
   - Target platform: Linux server

3. Constitution compliance was verified and all principles were satisfied:
   - AI-Native Design: Supports retrieval-first architecture
   - Source Grounding: Includes storage for book content and metadata
   - Beginner Clarity: Clear, well-commented code approach
   - Modularity: Isolated in /db modules
   - Spec-Driven Development: Following proper workflow
   - Determinism: Consistent storage/retrieval mechanisms
   - Safety: Secure credential handling

4. The project structure was defined with the database layer isolated at /backend/app/db with:
   - neon.py for Neon Serverless Postgres connector
   - qdrant.py for Qdrant Cloud connector
   - config.py for configuration
   - utils.py for utilities

5. API contracts were created for both Neon Postgres and Qdrant Cloud connectors, defining methods for all necessary operations.

6. A quickstart guide was created with instructions for environment setup, installation, and usage examples.

7. The agent context was updated with the new technologies from the plan.

8. All requirements from the original goal have been addressed while maintaining the constraints of not implementing ingestion or retrieval logic.

The plan provides a stable, isolated database layer foundation ready for future ingestion and RAG retrieval specifications.
```