# Implementation Plan: Retrieval-Enabled Agent with OpenAI Agents SDK + FastAPI

**Branch**: `005-retrieval-agent` | **Date**: 2025-12-11 | **Spec**: [link](spec.md)
**Input**: Feature specification from `/specs/005-retrieval-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan addresses the primary requirement to create a backend Agent that can answer questions about the book by integrating OpenAI Agents SDK with a custom retrieval tool that queries Qdrant and returns the most relevant chunks as context. The implementation will include a FastAPI backend with endpoints for user interaction, an OpenAI Agent with retrieval capability, and proper logging of the Agent → Tool → Agent handoffs. Based on research, the implementation will use the OpenAI Agents SDK, FastAPI framework, Qdrant for vector storage, and Cohere for embeddings, ensuring consistency with the existing system architecture.

## Technical Context

**Language/Version**: Python 3.11+ (consistent with existing backend)
**Primary Dependencies**: fastapi (web framework), openai (Agents SDK), qdrant-client (vector database), cohere (embeddings), pydantic (data validation), python-dotenv (configuration)
**Storage**: Qdrant Cloud (vector database with book embeddings), minimal local storage for session state
**Testing**: pytest (for API and agent functionality testing)
**Target Platform**: Linux/Windows/MacOS server environment (containerizable)
**Project Type**: Backend API service (single project with multiple modules)
**Performance Goals**: Process requests within 10 seconds, handle 100+ concurrent requests, maintain <500ms response time for simple queries
**Constraints**: Must work with existing Qdrant collection and Cohere API keys, responses must be grounded only in book content, all interactions must be logged
**Scale/Scope**: Support 1000+ daily active users, handle 10k+ questions per day, integrate with existing book content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle Alignment Check:
- ✅ **Accuracy**: The agent will provide responses exclusively based on book content retrieved from the vector database
- ✅ **Clarity**: The system will provide clear, understandable responses to user questions about the book
- ✅ **Depth & Rigor**: The implementation will use proper agent architecture with OpenAI Agents SDK and robust retrieval mechanisms
- ✅ **Traceability**: Each response will be tied back to specific book sections/chunks in the vector database
- ✅ **Spec-Driven Workflow**: Following the /sp.specify → /sp.plan → /sp.tasks → /sp.implementation workflow
- ✅ **Zero Ambiguity**: The system will have deterministic outputs with clear logging of Agent → Tool → Agent interactions

### Constraint Compliance Check:
- ✅ **Technology Stack Compliance**: Using FastAPI, OpenAI Agents SDK, Qdrant Cloud, and Cohere as specified
- ✅ **Zero Ambiguity**: Outputs will be deterministic with proper logging and traceability
- ✅ **Content Quality**: Responses will be grounded exclusively in book content with proper sourcing

### Gate Status: **PASSED** - Proceed to Phase 0 research

## Re-evaluation: Post-Design Constitution Check

### Updated Principle Alignment Check:
- ✅ **Accuracy**: The agent will provide responses exclusively based on book content retrieved from Qdrant with proper validation mechanisms
- ✅ **Clarity**: The system will provide clear, understandable responses with cited sources from the book content
- ✅ **Depth & Rigor**: The implementation uses proper agent architecture with OpenAI Agents SDK and robust retrieval mechanisms with Cohere embeddings
- ✅ **Traceability**: Each response will be tied back to specific book sections/chunks in the vector database with proper logging
- ✅ **Spec-Driven Workflow**: Following the /sp.specify → /sp.plan → /sp.tasks → /sp.implementation workflow
- ✅ **Zero Ambiguity**: The system will have deterministic outputs with structured data models and clear logging of Agent → Tool → Agent interactions

### Updated Constraint Compliance Check:
- ✅ **Technology Stack Compliance**: Using FastAPI, OpenAI Agents SDK, Qdrant Cloud, and Cohere as specified and consistent with existing system
- ✅ **Zero Ambiguity**: Outputs will be deterministic with proper logging and traceability through structured data models
- ✅ **Content Quality**: Responses will be grounded exclusively in book content with proper sourcing and verification mechanisms

### Post-Design Gate Status: **PASSED** - Ready for Phase 2 tasks

## Project Structure

### Documentation (this feature)

```text
specs/005-retrieval-agent/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
```text
backend/
├── src/
│   └── agent/
│       ├── main.py              # FastAPI application entry point
│       ├── agent_service.py     # Core OpenAI Agent implementation
│       ├── retrieval_tool.py    # Custom Qdrant retrieval tool
│       ├── models.py            # Pydantic data models
│       ├── config.py            # Configuration management
│       └── utils.py             # Utility functions
└── tests/
    ├── test_api.py             # API endpoint tests
    └── test_agent.py           # Agent functionality tests
```

**Structure Decision**: Using a dedicated `agent` module within the existing backend structure to maintain consistency with the rest of the system. The module will implement the FastAPI backend with OpenAI Agents SDK integration, and will reuse existing dependencies and configuration patterns from the previous components.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
