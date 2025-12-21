# Research Summary: Unified Agentic RAG Chatbot System

## Decision: Agent Router Implementation
**Rationale**: The agent router will be implemented as a lightweight classification and delegation layer that sits between the API endpoint and the existing RAG/LLM systems. This ensures no modifications to existing functionality while providing the required routing logic.

## Decision: Query Classification Logic
**Rationale**: Classification will follow a simple priority order:
1. If `selected_text` parameter is present in the request, use selected-text-only mode
2. If user explicitly specifies `mode=general`, use general knowledge mode
3. Otherwise, route to book RAG mode

This approach is straightforward to implement and understand.

## Decision: Technology Stack
**Rationale**: Using the existing technology stack (Python 3.11, FastAPI, Pydantic) ensures consistency with the existing codebase and reduces complexity. No new technologies need to be introduced.

## Decision: Response Format Standardization
**Rationale**: All response paths will be normalized to the unified format: `{ "answer": "string", "sources": [], "mode": "selected_text | book | general" }` to ensure consistent API behavior.

## Decision: Integration with Existing RAG Pipeline
**Rationale**: The agent will internally call the existing RAG pipeline functionality without duplicating code. This respects the constraint of not modifying existing systems while reusing their functionality.

## Alternatives Considered

### Alternative 1: Full rewrite of RAG pipeline
- Rejected because it violates the constraint of not modifying existing ingestion, RAG, Qdrant, Neon, or `/query` endpoint logic

### Alternative 2: Separate microservice for agent logic
- Rejected because it adds unnecessary complexity and infrastructure overhead for what is essentially a routing function

### Alternative 3: Client-side routing
- Rejected because it would require changes to the frontend and expose implementation details to the client