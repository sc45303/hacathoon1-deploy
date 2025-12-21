# Feature Specification: Book Ingestion Pipeline for RAG Chatbot

**Feature Branch**: `009-book-ingestion`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Book Ingestion Pipeline for RAG Chatbot Target: Build a controlled ingestion pipeline that reads book content from the existing Docusaurus project and prepares it for RAG usage. Audience: - AI code generation tools (Qwen CLI) - Backend developers extending ingestion or retrieval later Scope: This spec defines ONLY the ingestion pipeline. It does not include RAG querying, AI agents, or API endpoints. Objectives: - Read book content from Markdown source files - Split content into clean, reusable text chunks - Store raw text and metadata in Neon Postgres - Generate embeddings for each chunk - Store embeddings in Qdrant with correct payload linkage Data flow: Markdown files → text extraction → chunking → metadata tagging (book, chapter, section) → embedding generation → Postgres insert → Qdrant upsert Success criteria: - Book content can be ingested via a script (manual trigger) - Each text chunk is stored in Neon with metadata - Each chunk has a corresponding vector in Qdrant - No ingestion runs automatically on backend startup - Ingestion can be re-run safely (idempotent behavior) Constraints: - Use OpenAI embeddings API only - Respect Qdrant Free Tier limits - Chunk size must be configurable - No hardcoded file paths - No ingestion logic inside FastAPI routes - Script-based execution only Includes: - Markdown reader utility - Text chunking logic - Embedding generation module - Postgres insert logic - Qdrant upsert logic - Ingestion script under /scripts Not building: - RAG retrieval logic - AI agent reasoning - Chat endpoints - Frontend integration - Automatic ingestion triggers - File upload APIs Out of scope: - Advanced chunk optimization - Semantic re-ranking - Multilingual ingestion - Incremental updates - Deletion pipelines Timeline: - One focused iteration Validation: - Running the ingestion script processes book files - Postgres table contains inserted text chunks - Qdrant collection contains vectors with correct payload - Backend server still starts without running ingestion"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Backend Developer Ingests Book Content (Priority: P1)

A backend developer needs to run a controlled ingestion pipeline that reads book content from the existing Docusaurus Markdown files and prepares the content for RAG usage by storing text chunks in a structured database and vector embeddings in a vector database.

**Why this priority**: This is the foundational capability required for the RAG system to function. Without ingested content, the chatbot cannot retrieve relevant information from the book to generate responses.

**Independent Test**: A developer can run the ingestion script manually, and it successfully processes Markdown files, stores text chunks in the structured database with metadata, and corresponding embeddings in the vector database, all with proper linkage between the two systems.

**Acceptance Scenarios**:

1. **Given** the ingestion script is available and configured with proper credentials for the databases, **When** a developer runs the ingestion script with valid Markdown files as input, **Then** all text chunks are stored in the structured database with appropriate metadata and corresponding embeddings are stored in the vector database with proper payload linkage
2. **Given** the ingestion script has valid configuration and access to the Docusaurus project Markdown files, **When** the developer executes the script, **Then** the process completes without errors and can be verified through database queries

---

### User Story 2 - AI Code Generation Tool Analyzes Ingestion Process (Priority: P2)

An AI code generation tool (like Qwen CLI) needs to analyze the ingestion pipeline to understand the data flow, chunking approach, and storage schema to generate appropriate retrieval and processing code for future features.

**Why this priority**: The project follows spec-driven development workflow, meaning AI tools need to understand the ingestion implementation to generate appropriate retrieval methods and maintain consistency across the system.

**Independent Test**: An AI tool can examine the ingestion pipeline code and identify the defined schemas, chunking approach, and data flow patterns without confusion.

**Acceptance Scenarios**:

1. **Given** the ingestion pipeline implementation, **When** an AI tool analyzes the codebase, **Then** it can correctly identify the chunking approach, storage schemas, and data flow patterns
2. **Given** the environment configuration, **When** an AI generates related code, **Then** it uses the correct configuration parameters and follows the same patterns as the ingestion pipeline

---

### User Story 3 - System Maintainer Validates Ingestion Output (Priority: P3)

A system maintainer needs to verify that the ingestion process completed successfully and that the resulting data in both databases is consistent and properly linked.

**Why this priority**: Ensuring data integrity between the structured database and vector database is essential for system reliability and accuracy of the RAG responses.

**Independent Test**: A maintainer can verify that for each text chunk stored in the structured database, there is a corresponding embedding in the vector database with correct payload linkage.

**Acceptance Scenarios**:

1. **Given** a completed ingestion run, **When** a maintainer validates the database contents, **Then** the number of text chunks in the structured database matches the number of embeddings in the vector database and all have proper linkage
2. **Given** properly ingested content, **When** a maintainer queries the databases, **Then** the metadata in the structured database aligns with the payload data in the vector database

---

### Edge Cases

- What happens when Markdown files contain malformed content or special characters?
- How does the system handle very large Markdown files that might exceed service limits?
- What occurs if the embedding API is temporarily unavailable during embedding generation?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST read book content from Markdown source files in the Docusaurus project structure
- **FR-002**: System MUST split book content into configurable-sized text chunks suitable for embedding generation
- **FR-003**: System MUST store text chunks with metadata (book, chapter, section) in a structured database
- **FR-004**: System MUST generate embeddings for each text chunk using an AI embeddings service
- **FR-005**: System MUST store generated embeddings in a vector database with correct payload linkage to corresponding database records
- **FR-006**: System MUST provide an idempotent ingestion process that can be safely re-run without duplicating content
- **FR-007**: System MUST validate that no ingestion runs automatically on backend startup (script-only execution)
- **FR-008**: System MUST respect service usage limits during the ingestion process
- **FR-009**: System MUST store linkage information between database records and vector database entries to enable retrieval
- **FR-010**: System MUST accept configuration parameters for chunk size and file locations without hardcoded paths

### Key Entities

- **BookContentChunk**: Represents a segment of book text with associated metadata (book, chapter, section) and unique identifiers linking the database record to its corresponding vector in the vector database
- **EmbeddingVector**: Represents the vector embedding of a text chunk stored in a vector database, with payload containing references to the corresponding record in the structured database
- **IngestionRecord**: Represents the execution of an ingestion run, tracking which files were processed and when

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The ingestion script can be successfully triggered manually and completes the processing of all Markdown files within a reasonable time frame
- **SC-002**: Each text chunk from the book content is stored in the structured database with complete metadata (book, chapter, section) without data loss
- **SC-003**: Each text chunk has a corresponding vector embedding stored in the vector database with correct payload linkage to the structured database record
- **SC-004**: The backend server continues to start without triggering any ingestion processes automatically
- **SC-005**: The ingestion process can be re-run safely without duplicating content in either database (idempotent behavior)
- **SC-006**: The ingestion process operates within service usage limits during execution
- **SC-007**: Configuration parameters (chunk size, file locations) can be modified without code changes
