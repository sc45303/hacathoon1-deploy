<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Added sections: AI-Native Design, Source Grounding, Beginner Clarity, Modularity, Spec-Driven Development, Determinism, Safety principles
Removed sections: Physical AI & Humanoid Robotics specific principles
Templates requiring updates: ✅ No updates needed - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md, .qwen/commands/*.toml
Follow-up TODOs: None
Modified principles: Changed from Physical AI & Humanoid Robotics course to AI-Native Book with RAG Chatbot
-->
# Project Constitution: Unified AI-Native Book with Integrated RAG Chatbot

## Core Principles

### I. AI-Native Design
The system must follow a retrieval-first, generation-second architecture. All AI processing should prioritize retrieving relevant information from the book content before any generation occurs, ensuring that all responses are grounded in the book's content.

### II. Source Grounding
The chatbot must ONLY answer from book content. No external knowledge sources are allowed, and all responses must be directly tied to information present in the book to prevent hallucinations and maintain accuracy.

### III. Beginner Clarity
Every component must be understandable by a junior developer. All code, architecture decisions, and documentation must be clear, well-commented, and approachable to developers with limited experience in AI systems, RAG, or backend development.

### IV. Modularity
Each system layer must be cleanly separated. The frontend book, backend services, AI layer, and data storage must operate as distinct units with clear interfaces, allowing for independent development, testing, and maintenance.

### V. Spec-Driven Development
No direct coding without an approved spec. All development must follow the Spec-Kit Plus workflow: /sp.specify → Requirements, /sp.plan → Strategy, /sp.tasks → Task breakdown, /sp.implementation → Final output. This ensures consistency, traceability, and quality across all deliverables.

### VI. Determinism
Same input + same context → predictable output. The system must behave consistently, returning the same or equivalent responses when presented with identical queries and contextual information.

### VII. Safety
No hallucinated answers outside provided context. The system must be strictly constrained to generate responses based only on retrieved text from the book content, with appropriate responses when questions cannot be answered from available sources.

## System Components and Architecture

### Frontend Book
- Built with Docusaurus as a static site
- Already deployed and treated as read-only for this phase
- Hosts the embedded chatbot UI within the book pages

### Backend Services
- Located at /backend within the existing repository
- Built using FastAPI for high-performance, asynchronous operation
- Serves as the single source of truth for chatbot logic and data processing
- Must be deployable independently from the frontend book

### AI Layer
- OpenAI Agents / ChatKit SDK for intelligent query processing
- Used for embeddings and response generation
- Agents must be strictly constrained to retrieved context from the book
- Follow retrieval-first, generation-second design principles

### Data Layer
- Neon Serverless Postgres for storing book text, chapters, sections, and metadata
- Qdrant Cloud (Free Tier) for storing vector embeddings
- Qdrant used exclusively for semantic retrieval operations
- Connection pooling must be properly implemented for Neon Serverless

### RAG Logic
- Query → embedding → vector search workflow
- Retrieve relevant book chunks based on semantic similarity
- Construct bounded context from retrieved content
- Generate answer ONLY from retrieved text and provided context
- Handle selected-text-only answering capability

## Special Capabilities

### Selected-Text-Only Answering
When users provide selected text, the system must:
- Ignore the rest of the book content temporarily
- Answer strictly from the provided selected content
- Maintain all safety and grounding constraints within the selected context

## Standards and Technologies

### Technical Standards
- Python 3.11+ for all backend development
- FastAPI for HTTP layer implementation
- Pydantic for data validation and serialization
- Async-first design for optimal performance
- Environment variables for all secrets and configuration
- No hardcoded keys, credentials, or configuration values

### Deployment and Operation
- Backend must be deployable independently from the frontend
- Must respect Qdrant Free Tier limits and usage guidelines
- Proper error handling and graceful degradation when resources are limited
- Comprehensive logging and monitoring for debugging and maintenance

## Governance

This constitution governs all project activities and supersedes any conflicting practices. All contributions must align with these principles to maintain system quality and adherence to hackathon requirements. 

Amendments require documentation of rationale, approval by project maintainers, and migration plans for affected components. All specifications, plans, tasks, and implementations must explicitly verify compliance with these principles during development.

All pull requests and reviews must validate constitutional adherence before merging, ensuring the final hackathon submission meets all outlined requirements.

**Version**: 1.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-16