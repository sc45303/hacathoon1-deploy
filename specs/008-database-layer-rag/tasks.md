---

description: "Task list for database layer implementation"
---

# Tasks: Database Layer for RAG Chatbot (Neon + Qdrant)

**Input**: Design documents from `/specs/008-database-layer-rag/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below assume web app - adjust based on plan.md structure

<!--
  ============================================================================
  User Story tasks based on spec.md, research.md, data-model.md, and contracts/
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Prepare environment configuration - Update core/config.py to include: NEON_DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME
- [X] T002 Add database dependencies - Update backend/requirements.txt to include: asyncpg, qdrant-client
- [X] T003 [P] Create backend/app/db directory structure if it doesn't exist
- [X] T004 [P] Update .env.example with placeholders for Neon and Qdrant variables

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Create Neon Postgres connector module at backend/app/db/neon.py with async methods as per API contract
- [X] T006 Create Qdrant client connector module at backend/app/db/qdrant.py with async methods as per API contract
- [X] T007 Create utility functions module at backend/app/db/utils.py with connection validation utilities
- [X] T008 Define Postgres schema documentation in backend/app/db/neon.py as comments
- [X] T009 Define Qdrant collection schema documentation in backend/app/db/qdrant.py as comments

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Backend Developer Integrates Database Layer (Priority: P1) 🎯 MVP

**Goal**: Backend developers can initialize both Neon Postgres and Qdrant database connections to store and retrieve book content for the RAG chatbot system.

**Independent Test**: A developer can initialize both database connections without errors and verify the connection objects are properly created.

### Implementation for User Story 1

- [X] T010 [P] [US1] Implement lazy initialization in NeonDB class (init_connection_pool method) according to contract
- [X] T011 [P] [US1] Implement lazy initialization in QdrantDB class (init_client method) according to contract
- [X] T012 [US1] Implement all data operations in NeonDB (insert_chunk, get_chunks_by_book_id, get_chunk_by_id, update_chunk_content, delete_chunk) according to contract
- [X] T013 [US1] Implement all vector operations in QdrantDB (upload_vector, batch_upload_vectors, search_similar, get_vector_by_id, delete_vector) according to contract
- [X] T014 [US1] Implement all payload operations in QdrantDB (update_payload) according to contract
- [X] T015 [US1] Ensure no database queries are executed during application startup (lazy initialization)
- [X] T016 [US1] Create connection validation utilities in backend/app/db/utils.py (validate_connection and health_check methods)
- [X] T017 [US1] Add safety guards to prevent database connections from initializing if env vars missing

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - AI Code Generator Creates Database Schemas (Priority: P1)

**Goal**: AI code generation tools can analyze the database layer to understand the schema and create appropriate data access patterns for future features.

**Independent Test**: An AI tool can examine the database layer and identify the defined schemas and their relationships without confusion.

### Implementation for User Story 2

- [X] T018 [P] [US2] Document table schema for book_chunks in backend/app/db/neon.py (per data-model.md)
- [X] T019 [P] [US2] Document collection schema for book_chunks_vectors in backend/app/db/qdrant.py (per data-model.md)
- [X] T020 [US2] Ensure all API contracts (methods and signatures) are clearly documented in both neon.py and qdrant.py
- [X] T021 [US2] Add clear schema definitions as constants or types in backend/app/db modules
- [X] T022 [US2] Document entity relationships in backend/app/db/__init__.py or README

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - System Maintainer Validates Database Connections (Priority: P2)

**Goal**: System maintainers can validate that database connections are properly established and secure before deploying the system.

**Independent Test**: A maintainer can use connection validation utilities to verify that both databases are accessible and properly configured.

### Implementation for User Story 3

- [X] T023 [P] [US3] Implement validate_connection method in NeonDB class with proper error handling
- [X] T024 [P] [US3] Implement validate_connection method in QdrantDB class with proper error handling
- [X] T025 [US3] Implement health_check method in NeonDB class with proper response format
- [X] T026 [US3] Implement health_check method in QdrantDB class with proper response format
- [X] T027 [US3] Create connection validation utility function in backend/app/db/utils.py that validates both databases
- [X] T028 [US3] Create comprehensive health check utility in backend/app/db/utils.py that checks both databases
- [X] T029 [US3] Ensure appropriate error messages are returned for misconfigured systems

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T030 [P] Update backend/README.md to document database setup requirements and explain Neon and Qdrant roles
- [X] T031 [P] Include notes about Free Tier limitations in documentation
- [X] T032 Validate imports - ensure neon.py and qdrant.py can be imported without errors
- [X] T033 Verify FastAPI app still starts without triggering DB logic
- [X] T034 Manual validation - start server, open Python REPL, instantiate Neon and Qdrant clients, confirm no runtime exceptions
- [X] T035 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all parallel components for User Story 1 together:
Task: "Implement lazy initialization in NeonDB class (init_connection_pool method) according to contract"
Task: "Implement lazy initialization in QdrantDB class (init_client method) according to contract"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence