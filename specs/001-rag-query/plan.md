# Implementation Plan: RAG Query System

**Branch**: `001-rag-query` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-query/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a Retrieval Augmented Generation (RAG) system that allows users to submit natural language questions about book content and receive accurate, source-grounded answers. The system will retrieve relevant content from Qdrant vector database, fetch full text from Neon Postgres, construct a strict prompt, and generate responses using OpenAI Chat Completion with hallucination prevention.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI SDK, Qdrant client, asyncpg (for Neon Postgres)
**Storage**: Neon Postgres (book_content_chunks), Qdrant vector database
**Testing**: pytest for unit and integration tests
**Target Platform**: Linux server (Vercel deployment)
**Project Type**: Web backend service with API endpoints
**Performance Goals**: <10 second response time for 95% of queries
**Constraints**: <100ms internal processing time (not including external API calls), async throughout
**Scale/Scope**: Support up to 100 concurrent users, handle 10k questions per day

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ AI-Native Design: Follows retrieval-first, generation-second architecture
- ✅ Source Grounding: Responses will be strictly based on book content only
- ✅ Beginner Clarity: Each component will have clear interfaces and documentation
- ✅ Modularity: Separate modules for retrieval, DB access, prompting, and generation
- ✅ Spec-Driven Development: Following SpecKit Plus workflow
- ✅ Determinism: Same inputs will produce consistent outputs
- ✅ Safety: Hallucination prevention mechanisms built into prompt and response generation

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-query/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── query-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── api/
│   │   └── routes/
│   │       └── query.py    # POST /query endpoint
│   ├── rag/
│   │   ├── retriever.py    # Qdrant similarity search
│   │   ├── prompt.py       # RAG prompt template
│   │   ├── generator.py    # LLM answer generation
│   │   └── pipeline.py     # End-to-end RAG orchestration
│   └── db/
│       └── neon.py         # Database access layer (reuse existing methods)
├── main.py
└── requirements.txt
```

**Structure Decision**: Web application structure with backend service containing the RAG functionality and API endpoints.

## Implementation Phases

### Phase 1 — Retrieval Layer (Qdrant)

Create `app/rag/retriever.py` with:

1. **Async Similarity Search Function**:
   - Input: query text, top_k parameter (default 5)
   - Process: Convert query to embedding using the same model as ingestion, perform cosine similarity search in Qdrant
   - Output: List of dictionaries containing chunk_id, chunk_hash, similarity score, and metadata (book_id, chapter, section, source_file)

2. **Top-K Retrieval Strategy**:
   - Retrieve top-K most relevant vectors based on similarity score
   - Default K=5, but allow configuration
   - Sort results by similarity score in descending order

3. **Payload Normalization**:
   - Normalize all retrieved payloads to a consistent format
   - Extract required fields: chunk_id, chunk_hash, similarity_score, metadata

4. **Error Handling**:
   - Handle Qdrant unavailability gracefully
   - Return empty results with appropriate logging if Qdrant is down

### Phase 2 — Structured Context Fetch (Neon)

Update `app/db/neon.py` functions and integrate in `app/rag/pipeline.py`:

1. **Chunk Fetching**:
   - Function to fetch full chunk text by chunk_id
   - Handle multiple chunk_ids in a single database query for efficiency

2. **Order Preservation**:
   - Maintain the original order of chunks as returned by the retriever
   - Use the original ordering to maintain coherence in the context

3. **Deduplication**:
   - Safely deduplicate chunks to prevent redundant information
   - Use chunk_id for deduplication (not content-based)
   - Preserve order of first occurrence

4. **Missing Chunk Handling**:
   - Gracefully handle missing or deleted chunks
   - Log warnings and continue processing with available chunks

### Phase 3 — Prompt Engineering

Create `app/rag/prompt.py` with:

1. **Strict RAG Prompt Template**:
   - System message that enforces using ONLY retrieved context
   - Clear instructions to avoid hallucinations
   - Template for formatting retrieved context into the prompt
   - Fallback instruction to return "Answer not found in provided content" if context is insufficient

2. **Reusable Prompt Constructor**:
   - Function that takes retrieved context and user question
   - Builds a properly formatted prompt for the LLM
   - Ensures consistent formatting across all requests

### Phase 4 — Answer Generation

Create `app/rag/generator.py` with:

1. **OpenAI Integration**:
   - Async function to call OpenAI Chat Completion API
   - Use low temperature (≤ 0.3) to minimize creative variance
   - Proper system and user message formatting

2. **Response Processing**:
   - Extract the answer text from the API response
   - Handle API errors gracefully
   - Return clean text without markdown if possible

3. **Error Handling**:
   - Handle OpenAI API failures
   - Implement appropriate timeouts
   - Return descriptive errors when LLM service is unavailable

### Phase 5 — RAG Orchestration

Update `app/rag/pipeline.py` with:

1. **End-to-End Pipeline Function**:
   - Combine retrieval, database fetch, prompt construction, and generation
   - Handle the complete RAG flow: query → embedding → retrieval → DB fetch → prompting → generation
   - Return answer text and source metadata

2. **Stateless Design**:
   - Ensure the pipeline function is stateless
   - All dependencies are passed as parameters or accessed through services

3. **Response Formatting**:
   - Format the final response with answer text and sources array
   - Sources should contain source_file, chapter, and section information

### Phase 6 — API Layer

Create `app/api/routes/query.py` with:

1. **POST /query Endpoint**:
   - Accept JSON input with "question" field
   - Validate input using Pydantic models
   - Call the RAG pipeline function
   - Format and return response as specified

2. **Input Validation**:
   - Ensure question is a non-empty string
   - Validate request structure

3. **Response Formatting**:
   - Format response as JSON with "answer" and "sources" fields
   - Ensure sources array has proper structure with source_file, chapter, section

4. **Error Handling**:
   - Handle and return appropriate HTTP status codes
   - Include descriptive error messages in responses

5. **Logging**:
   - Log retrieval count
   - Log chunk IDs used
   - Log token usage if available
   - Log error categories (retrieval / DB / LLM)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| (No violations found) | | |