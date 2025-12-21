---

description: "Task list for Unified Agentic RAG Chatbot System implementation"
---

# Tasks: Unified Agentic RAG Chatbot System

**Input**: Design documents from `/specs/010-agentic-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/app/` at repository root
- Paths shown below based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create agent router directory structure in backend/app/api/routes/
- [x] T002 Verify required dependencies are available (FastAPI, Pydantic, OpenAI SDK, Qdrant client, Neon Postgres driver)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T003 Create Pydantic models for agent requests/responses in backend/app/models/agent.py
- [x] T004 [P] Create agent service module in backend/app/services/agent_service.py
- [x] T005 [P] Create agent router endpoint in backend/app/api/routes/agent.py
- [x] T006 Register agent router in backend/app/api/main.py
- [x] T007 Verify existing RAG pipeline is accessible for delegation

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask questions about book content using RAG (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content and receive accurate answers with sources

**Independent Test**: Can be fully tested by sending a question about book content to the `/agent/query` endpoint and verifying that the response contains an answer with sources and the mode is set to "book".

### Implementation for User Story 1

- [x] T008 [US1] Implement book RAG mode detection in agent service
- [x] T009 [US1] Integrate with existing RAG pipeline for book queries in backend/app/services/agent_service.py
- [x] T010 [US1] Ensure sources are preserved from existing RAG pipeline in responses
- [x] T011 [US1] Test book RAG query routing and response format
- [x] T012 [US1] Validate that existing `/query` endpoint remains unchanged

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Ask questions based on selected text only (Priority: P1)

**Goal**: Enable users to ask questions about specific text they've selected from a book and receive answers only from that text

**Independent Test**: Can be fully tested by sending a question with selected_text parameter to the `/agent/query` endpoint and verifying that the response is generated only from the provided text.

### Implementation for User Story 2

- [x] T013 [US2] Implement selected text mode detection in agent service
- [x] T014 [US2] Create selected-text-only answer generator in backend/app/services/agent_service.py
- [x] T015 [US2] Implement strict prompt rules: "answer only from provided text"
- [x] T016 [US2] Ensure no external RAG sources are accessed in selected-text mode
- [x] T017 [US2] Test selected text query routing and response format
- [x] T018 [US2] Validate adherence to only provided text without hallucinating information from other sources

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Ask general questions not related to the book (Priority: P2)

**Goal**: Enable users to ask general questions that are not related to the book and receive answers from the LLM

**Independent Test**: Can be fully tested by sending a general knowledge question to the `/agent/query` endpoint and verifying that the response comes from the LLM without using RAG sources.

### Implementation for User Story 3

- [x] T019 [US3] Implement general knowledge mode detection in agent service
- [x] T020 [US3] Create general answer generator using direct LLM completion in backend/app/services/agent_service.py
- [x] T021 [US3] Ensure empty sources array for general knowledge responses
- [x] T022 [US3] Test general knowledge query routing and response format

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Agent Router Logic & Integration

**Goal**: Implement the core routing logic that classifies and routes queries appropriately

### Implementation for Router Logic

- [x] T023 [P] Implement query classification logic in backend/app/services/agent_service.py
- [x] T024 [P] Implement routing mechanism based on classification results
- [x] T025 [P] Create unified response formatter to ensure consistent output structure
- [x] T026 [P] Add mode field to all responses to indicate how the answer was generated
- [x] T027 [P] Add proper validation for all input parameters in request models

**Checkpoint**: Agent router logic fully integrated and functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T028 [P] Add comprehensive logging for agent operations in backend/app/services/agent_service.py
- [x] T029 [P] Add error handling and validation for edge cases (long selected_text, empty inputs, etc.)
- [x] T030 [P] Add unit tests for agent service in backend/tests/
- [x] T031 [P] Add integration tests for agent endpoint in backend/tests/api/test_agent.py
- [x] T032 [P] Update documentation with usage examples from quickstart.md
- [x] T033 [P] Performance testing to ensure <5 second response for all query types
- [x] T034 [P] Verify no modifications to existing ingestion, RAG, Qdrant, Neon, or `/query` endpoint logic
- [x] T035 [P] Run manual API testing with examples from quickstart.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Router Logic (Phase 6)**: Can be implemented in parallel with user stories
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
- Router Logic phase can be worked on in parallel with user stories
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all components for User Story 1 together:
Task: "Implement book RAG mode detection in agent service"
Task: "Integrate with existing RAG pipeline for book queries in backend/app/services/agent_service.py"
Task: "Ensure sources are preserved from existing RAG pipeline in responses"
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
5. Add Router Logic → Test integration → Deploy/Demo
6. Add Polish → Test all scenarios → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Book RAG queries)
   - Developer B: User Story 2 (Selected text queries)
   - Developer C: User Story 3 (General knowledge queries)
   - Developer D: Router Logic & Integration
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence