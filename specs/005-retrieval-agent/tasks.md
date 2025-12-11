# Implementation Tasks: Retrieval-Enabled Agent with OpenAI Agents SDK + FastAPI

**Feature**: 005-retrieval-agent | **Date**: 2025-12-11
**Input**: Feature spec, implementation plan, data model, contracts, research

**Strategy**: Build the agent service incrementally, starting with basic FastAPI setup and data models, then adding the retrieval tool, implementing the OpenAI Agent, and finally creating the API endpoints. Focus on the highest priority user story first (asking book-related questions) before moving to supporting stories.

## Dependencies

- **User Story 2** (health checks) and **User Story 3** (agent runs) can be developed in parallel after foundational tasks are complete
- All user stories depend on foundational setup and infrastructure tasks
- The agent service depends on the retrieval tool being implemented

## Parallel Execution Examples

Per Story 1:
- [P] T010-T012 can be worked on in parallel with different components
- [P] T013-T015 can be developed in parallel with API endpoints

Per Story 2:
- [P] T016-T017 can be developed in parallel with monitoring components

Per Story 3:
- [P] T018-T019 can be developed in parallel with advanced agent features

## Implementation Strategy

- **MVP Scope**: Implement basic `/ask` endpoint with agent functionality (User Story 1) with minimal health checks
- **Incremental Delivery**: Each user story builds on the previous one, adding more sophisticated features
- **Independent Testing**: Each user story can be tested independently after its completion

---

## Phase 1: Setup

**Goal**: Establish project structure, dependencies, and configuration for the agent service

- [x] T001 Create agent module structure in backend/src/agent/
- [x] T002 Install and configure dependencies (fastapi, openai, qdrant-client, cohere, pydantic, python-dotenv)
- [x] T003 Set up configuration loading from environment variables using python-dotenv

---

## Phase 2: Foundational Tasks

**Goal**: Implement core infrastructure components needed by all user stories

- [x] T004 Create data models in backend/src/agent/models.py (UserQuery, RetrievedChunk, AgentResponse, AgentSession, APIRequest, APIResponse)
- [x] T005 Create configuration management in backend/src/agent/config.py
- [x] T006 Implement utility functions in backend/src/agent/utils.py (for logging, ID generation, etc.)
- [x] T007 Set up logging to capture Agent → Tool → Agent interactions

---

## Phase 3: User Story 1 - Ask Book-Related Questions (Priority: P1)

**Goal**: Enable users to ask questions about the book and receive accurate, sourced answers

**Independent Test**: Can be fully tested by submitting a question through the `/ask` endpoint and verifying that the response is relevant to the book content and properly sourced.

- [x] T008 [US1] Implement retrieval tool in backend/src/agent/retrieval_tool.py that queries Qdrant using Cohere embeddings
- [x] T009 [US1] Create OpenAI agent service in backend/src/agent/agent_service.py with the retrieval tool integration
- [x] T010 [US1] Implement the /ask endpoint in the FastAPI app to process user queries
- [x] T011 [US1] Ensure retrieved chunks are properly injected into the Agent's context before answering
- [x] T012 [US1] Validate that responses are grounded in retrieved book content
- [x] T013 [US1] Return sourced answers with references to the retrieved chunks
- [x] T014 [US1] Add request/response validation using Pydantic models
- [x] T015 [US1] Implement timeout mechanism to ensure responses within 10 seconds

---

## Phase 4: User Story 2 - Verify Agent System Health (Priority: P2)

**Goal**: Enable system administrators to check the health status of the retrieval-enabled agent

**Independent Test**: Can be tested by calling the `/health` endpoint and verifying it returns an appropriate status response.

- [x] T016 [US2] Implement the /health endpoint in backend/src/agent/main.py
- [x] T017 [US2] Check the status of OpenAI, Qdrant, and Cohere services and return detailed health information

---

## Phase 5: User Story 3 - Execute Agent Runs Programmatically (Priority: P3)

**Goal**: Enable developers to programmatically execute agent runs with custom instructions

**Independent Test**: Can be tested by calling the `/agent/run` endpoint with appropriate parameters and verifying the agent executes as expected.

- [x] T018 [US3] Implement the /agent/run endpoint in backend/src/agent/main.py for programmatic agent execution
- [x] T019 [US3] Extend agent service to support custom instructions and parameters
- [x] T020 [US3] Return detailed output with tool call information in the response

---

## Phase 6: Main Application & Integration

**Goal**: Create main FastAPI application and integrate all components

- [x] T021 Create main FastAPI application in backend/src/agent/main.py
- [x] T022 Integrate all endpoints (/ask, /health, /agent/run) with their respective handlers
- [x] T023 Add proper error handling and status codes per the API contract
- [x] T24 Add rate limiting for the endpoints

---

## Phase 7: Testing & Validation

**Goal**: Add tests to verify the agent functionality works as expected

- [x] T025 Create unit tests for data models in backend/tests/test_models.py
- [x] T026 Create tests for the retrieval tool functionality in backend/tests/test_retrieval_tool.py
- [x] T027 Create API endpoint tests in backend/tests/test_api.py
- [x] T028 Create agent functionality tests in backend/tests/test_agent.py
- [x] T029 Test that 90%+ of queries return accurate, sourced responses

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Address edge cases, add documentation, and finalize the implementation

- [x] T030 Handle edge case when Qdrant service is unavailable during a query
- [x] T031 Handle edge case when there are no relevant matches in book content for a query
- [x] T032 Handle edge case when there are issues with OpenAI Agents SDK during execution
- [x] T033 Implement proper handling of multiple simultaneous user requests
- [x] T034 Add comprehensive documentation for the agent service
- [x] T035 Perform end-to-end testing with sample queries to confirm tool call → retrieval → final response flow