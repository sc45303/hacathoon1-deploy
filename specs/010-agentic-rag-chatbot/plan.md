# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Pydantic, OpenAI SDK, Qdrant client, Neon Postgres driver
**Storage**: Neon Serverless Postgres for book content and metadata, Qdrant Cloud for vector embeddings
**Testing**: pytest for backend tests, manual validation of agent routing logic
**Target Platform**: Linux server (backend service)
**Project Type**: Web application (backend service with API endpoints)
**Performance Goals**: <5 second response time for all query types, maintain existing RAG performance for book queries
**Constraints**: Must not modify existing ingestion, RAG, Qdrant, Neon, or `/query` endpoint logic; agent layer acts as wrapper only; must reuse existing RAG pipeline internally
**Scale/Scope**: Single endpoint POST `/agent/query` supporting three query modes (selected_text, book, general)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Gate 1: AI-Native Design
✅ PASS: The agent router follows retrieval-first, generation-second architecture by delegating to existing RAG pipeline for book content and using direct LLM completion only for general questions.

### Gate 2: Source Grounding
✅ PASS: The design ensures answers are properly sourced - for book queries via RAG, selected text queries only from provided text, and general queries marked as such. No external knowledge sources are used.

### Gate 3: Beginner Clarity
✅ PASS: The agent router logic is straightforward - classify query type and route to appropriate strategy. Well-documented code will make this approachable for junior developers.

### Gate 4: Modularity
✅ PASS: The agent layer is cleanly separated as a new component that orchestrates existing functionality without modifying underlying systems.

### Gate 5: Spec-Driven Development
✅ PASS: Following the Spec-Kit Plus workflow as required by this constitution.

### Gate 6: Determinism
✅ PASS: For the same input and context, the system will behave consistently. RAG queries will return equivalent results, selected text will always be processed the same way, and general queries will use the same LLM behavior.

### Gate 7: Safety
✅ PASS: The system is constrained to generate responses based on retrieved content for book queries, provided selected text for selected text queries, and appropriate responses when questions cannot be answered from available sources.

### Post-Design Re-evaluation
All constitutional gates continue to pass after Phase 1 design implementation. The data models, API contracts, and system architecture align with the project constitution requirements.

## Project Structure

### Documentation (this feature)

```text
specs/010-agentic-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── agent-api.yaml   # OpenAPI specification for the agent endpoint
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── api/
│   │   ├── routes/
│   │   │   └── agent.py    # New agent router endpoint
│   │   └── main.py         # Registration of new endpoint
│   ├── models/
│   │   └── agent.py        # Pydantic models for agent requests/responses
│   ├── services/
│   │   └── agent_service.py # Agent router logic and classification
│   └── core/
└── tests/
    └── api/
        └── test_agent.py   # Tests for the agent endpoint
```

**Structure Decision**: Web application backend structure selected, with the agent router implemented as a new endpoint in the existing backend service. This maintains modularity while integrating cleanly with the existing codebase.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
