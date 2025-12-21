---

description: "Task list for book ingestion pipeline implementation"
---

# Tasks: Book Ingestion Pipeline for RAG Chatbot

**Input**: Design documents from `/specs/009-book-ingestion/`
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

- [X] T001 [P] Create backend/app/ingestion/ directory structure
- [X] T002 [P] Add __init__.py files to backend/app/ingestion/
- [X] T003 [P] Create backend/scripts/ directory
- [X] T004 [P] Update .env.example with ingestion configuration variables: BOOK_SOURCE_DIR, CHUNK_SIZE, CHUNK_OVERLAP, OPENAI_EMBEDDING_MODEL

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Install required dependencies: python-markdown, openai, qdrant-client
- [X] T006 Ensure ingestion uses environment variables only (no hardcoded paths)
- [X] T007 Verify backend server starts without triggering ingestion (verify constraint from spec)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Backend Developer Ingests Book Content (Priority: P1) 🎯 MVP

**Goal**: Backend developer can run a controlled ingestion pipeline that reads book content from Markdown files and stores text chunks in structured database with metadata and corresponding embeddings in vector database.

**Independent Test**: A developer can run the ingestion script manually, and it successfully processes Markdown files, stores text chunks in the structured database with metadata, and corresponding embeddings in the vector database, all with proper linkage between the two systems.

### Implementation for User Story 1

- [X] T008 [P] [US1] Create reader.py module in backend/app/ingestion/ with read_markdown_files function per interface contract
- [X] T009 [P] [US1] Create reader.py with extract_text_from_markdown function per interface contract
- [X] T010 [P] [US1] Create chunker.py module in backend/app/ingestion/ with chunk_text function per interface contract
- [X] T011 [P] [US1] Create chunker.py with calculate_tokens function per interface contract
- [X] T012 [P] [US1] Create embedder.py module in backend/app/ingestion/ with generate_embedding function per interface contract
- [X] T013 [P] [US1] Create embedder.py with batch_generate_embeddings function per interface contract
- [X] T014 [US1] Create storage.py module in backend/app/ingestion/ with store_chunk function per interface contract
- [X] T015 [US1] Create storage.py with upsert_embedding function per interface contract
- [X] T016 [US1] Create storage.py with chunk_exists function for idempotency per interface contract
- [X] T017 [US1] Create scripts/ingest_book.py with command line interface per contract
- [X] T018 [US1] Implement ingestion orchestration in scripts/ingest_book.py following the process steps
- [X] T019 [US1] Implement idempotency using hash-based approach (per research decision)
- [X] T020 [US1] Add logging at each step of the ingestion process
- [X] T021 [US1] Verify no ingestion runs on FastAPI startup (per spec constraint)
- [X] T022 [US1] Add support for dry-run mode (no DB writes) to ingestion script

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - AI Code Generation Tool Analyzes Ingestion Process (Priority: P2)

**Goal**: AI code generation tools can analyze the ingestion pipeline to understand the data flow, chunking approach, and storage schema to generate appropriate retrieval and processing code for future features.

**Independent Test**: An AI tool can examine the ingestion pipeline code and identify the defined schemas, chunking approach, and data flow patterns without confusion.

### Implementation for User Story 2

- [X] T023 [P] [US2] Document the chunking approach in backend/app/ingestion/chunker.py with clear comments
- [X] T024 [P] [US2] Document storage schemas in backend/app/ingestion/storage.py with clear comments
- [X] T025 [US2] Ensure data flow patterns are clear in scripts/ingest_book.py with comments
- [X] T026 [US2] Make configuration parameters and patterns discoverable in code

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - System Maintainer Validates Ingestion Output (Priority: P3)

**Goal**: System maintainer can verify that the ingestion process completed successfully and that the resulting data in both databases is consistent and properly linked.

**Independent Test**: A maintainer can verify that for each text chunk stored in the structured database, there is a corresponding embedding in the vector database with correct payload linkage.

### Implementation for User Story 3

- [X] T027 [US3] Implement storage.py with update_ingestion_status function per interface contract
- [X] T028 [US3] Create ingestion_runs table in database per data-model.md
- [X] T029 [US3] Store linkage information between database records and vector database entries per FR-009
- [X] T030 [US3] Log total files, chunks, and vectors processed during ingestion
- [X] T031 [US3] Implement verification function to check data consistency between databases

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T032 [P] Update backend/README.md to document ingestion setup requirements
- [X] T033 [P] Include notes about Qdrant Free Tier limitations in documentation
- [X] T034 Validate that backend server still starts without triggering ingestion
- [X] T035 Verify rows exist in Neon dashboard after running ingestion
- [X] T036 Verify vectors exist in Qdrant dashboard after running ingestion
- [X] T037 Re-run script to confirm no duplication (idempotency)
- [X] T038 Run quickstart.md validation

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all parallel components for User Story 1 together:
Task: "Create reader.py module in backend/app/ingestion/ with read_markdown_files function per interface contract"
Task: "Create reader.py with extract_text_from_markdown function per interface contract"
Task: "Create chunker.py module in backend/app/ingestion/ with chunk_text function per interface contract"
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