# Implementation Tasks: RAG Query System

**Feature**: RAG Query System  
**Branch**: `001-rag-query` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)  
**Plan**: [plan.md](./plan.md) | **Dependencies**: [dependencies.md](./dependencies.md)

## Summary

This document contains ordered tasks to implement the Retrieval Augmented Generation (RAG) system that allows users to submit natural language questions about book content and receive accurate, source-grounded answers. The system retrieves relevant content from Qdrant vector database, fetches full text from Neon Postgres, constructs a strict prompt, and generates responses using OpenAI Chat Completion with hallucination prevention.

## Dependencies

- **User Story 2** (Receive Source Citations) depends on **User Story 1** (Query Book Content) completion
- **User Story 3** (Handle Various Question Types) depends on **User Story 1** and **User Story 2** completion
- All user stories depend on foundational tasks completion

## Implementation Strategy

1. **MVP Scope**: Complete User Story 1 (Query Book Content) to deliver core functionality
2. **Incremental Delivery**: Add source citations (US2), then question type handling (US3)
3. **Parallel Execution**: Tasks marked with [P] can be executed in parallel
4. **Independent Testing**: Each user story can be tested independently after completion

## Phase 1: Setup

- [X] T001 Set up project structure in backend/app/ with rag/ and api/ subdirectories
- [X] T002 Configure environment variables for OpenAI, Qdrant, and Neon Postgres connections
- [X] T003 Install dependencies: fastapi, openai, qdrant-client, asyncpg, pydantic, python-dotenv

## Phase 2: Foundational Components

- [X] T004 [P] Create embedding utility function in backend/app/utils/embeddings.py to convert text to embeddings using OpenAI model
- [X] T005 [P] Verify existing Qdrant client connection in backend/app/core/qdrant.py
- [X] T006 [P] Verify existing Neon Postgres client connection in backend/app/core/database.py

## Phase 3: User Story 1 - Query Book Content (Priority: P1)

**Goal**: Enable a user to ask a natural language question and receive an accurate answer based on ingested book content

**Independent Test**: Can be fully tested by submitting various questions to the `/query` endpoint and verifying that responses are accurate, grounded in the book content, and include proper citations.

### 3.1 Implementation Tasks

- [X] T007 [P] [US1] Create vector retriever in backend/app/rag/retriever.py with async similarity search
- [X] T008 [P] [US1] Implement Top-K retrieval (default K=5) with cosine similarity in retriever.py
- [X] T009 [P] [US1] Add normalization of retrieval results (chunk_id, score, metadata) in retriever.py
- [X] T010 [P] [US1] Add error handling for empty results in retriever.py
- [X] T011 [US1] Update Neon DB module in backend/app/db/neon.py to fetch text chunks by chunk_id
- [X] T012 [US1] Implement chunk deduplication by chunk_id in neon.py
- [X] T013 [US1] Preserve original ranking order of chunks in neon.py
- [X] T014 [US1] Handle missing records safely by logging and skipping in neon.py
- [X] T015 [US1] Create RAG prompt template in backend/app/rag/prompt.py with strict context-only instructions
- [X] T016 [US1] Add hallucination prevention mechanisms to prompt.py
- [X] T017 [US1] Implement fallback message for insufficient context in prompt.py
- [X] T018 [P] [US1] Create answer generator in backend/app/rag/generator.py with OpenAI chat completion
- [X] T019 [P] [US1] Set temperature ≤ 0.3 in generator.py for minimal creative variance
- [X] T020 [P] [US1] Add timeout and error handling in generator.py
- [X] T021 [US1] Create RAG pipeline in backend/app/rag/pipeline.py combining all components
- [X] T022 [US1] Implement end-to-end RAG flow in pipeline.py: query→embedding→retrieval→DB fetch→prompting→generation
- [X] T023 [US1] Format response with answer text and source metadata in pipeline.py
- [X] T024 [US1] Create POST /query endpoint in backend/app/api/routes/query.py
- [X] T025 [US1] Add Pydantic validation for query requests in query.py
- [X] T026 [US1] Format structured JSON response per API contract in query.py
- [X] T027 [US1] Add proper HTTP error handling in query.py

## Phase 4: User Story 2 - Receive Source Citations (Priority: P2)

**Goal**: Provide users with source citations to verify the accuracy and trustworthiness of generated answers

**Independent Test**: Can be tested by submitting questions and verifying that the response includes a sources array with appropriate book_id, chapter, section, and source_file information.

### 4.1 Implementation Tasks

- [X] T028 [US2] Update pipeline.py to return source metadata (source_file, chapter, section) with answers
- [X] T029 [US2] Ensure source array structure matches API contract in query.py
- [X] T030 [US2] Test source citation accuracy with known content queries

## Phase 5: User Story 3 - Handle Various Question Types (Priority: P3)

**Goal**: Handle different types of natural language questions including factual, conceptual, and comparative queries

**Independent Test**: Can be tested by submitting different types of questions (factual, conceptual, comparative) and verifying that the system appropriately retrieves relevant content and generates accurate answers.

### 5.1 Implementation Tasks

- [X] T031 [US3] Test system with various factual question types to ensure accurate retrieval
- [X] T032 [US3] Test system with conceptual questions requiring synthesis of multiple sections
- [X] T033 [US3] Test system with comparative questions pulling information from different content parts
- [X] T034 [US3] Optimize retrieval for different question types if needed

## Phase 6: Logging & Cross-Cutting Concerns

- [X] T035 Add logging for retrieval count in pipeline.py
- [X] T036 Add logging for chunk IDs used in pipeline.py
- [X] T037 Add logging for empty context scenarios in pipeline.py
- [X] T038 Add logging for LLM failures in generator.py
- [X] T039 Ensure stateless pipeline with no global state in pipeline.py
- [X] T040 Verify Vercel-compatible async behavior in all modules
- [X] T041 Confirm queries work with real data from existing ingestion pipeline
- [X] T042 Verify no hallucinations in generated answers
- [X] T043 Confirm out-of-scope questions return fallback message
- [X] T044 Ensure API is production-ready with proper error handling

## Parallel Execution Examples

The following tasks can be executed in parallel:
- T004, T005, T006: Infrastructure setup tasks
- T007, T008, T009, T010: Retriever implementation tasks
- T018, T019, T020: Generator implementation tasks