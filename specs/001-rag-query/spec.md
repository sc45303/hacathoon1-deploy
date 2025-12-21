# Feature Specification: RAG Query System

**Feature Branch**: `001-rag-query`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "You are SpecKitPlus acting as a senior backend + RAG architect. Project Context: - Spec-3 (Ingestion) is COMPLETE and WORKING. - Markdown content has been chunked and stored in: - Neon Postgres (table: book_content_chunks) - Qdrant (collection already exists and contains embeddings) - Backend is FastAPI (Python, async). - OpenAI embeddings are already configured and working. - This project will be deployed on Vercel (no GitHub Pages). Your task: DEFINE SPEC-4 in detail (RAG Query System). ### SPEC-4 GOAL Enable a user to ask a natural language question and receive an accurate answer generated ONLY from ingested book content using Retrieval Augmented Generation (RAG). --- ### FUNCTIONAL REQUIREMENTS 1. Query Embedding - Convert user question into an embedding using the SAME OpenAI embedding model used in ingestion. - Must be async and reusable. 2. Vector Retrieval (Qdrant) - Perform similarity search in Qdrant. - Retrieve Top-K most relevant vectors (default K = 5). - Extract: - chunk_id - chunk_hash - similarity score - metadata (book_id, chapter, section, source_file) 3. Structured Data Fetch (Neon) - Fetch full chunk text from Neon Postgres using chunk_id. - Preserve original ordering where possible. - Deduplicate chunks safely. 4. Prompt Construction (RAG Prompt) - Build a strict prompt that: - Uses ONLY retrieved context - Forbids hallucination - Returns \u201cAnswer not found in provided content\u201d if context is insufficient - Prompt must be deterministic and reusable. 5. Answer Generation (LLM) - Generate final answer using OpenAI Chat Completion. - Temperature must be low (\u2264 0.3). - Output must be clean text (no markdown hallucinations). 6. API Endpoint - Create a POST endpoint: `/query` - Input: { \"question\": \"string\" } Output: { \"answer\": \"string\", \"sources\": [ { \"source_file\": \"string\", \"chapter\": \"string\", \"section\": \"string\" } ] } NON-FUNCTIONAL REQUIREMENTS Fully async (no blocking calls) Production-ready logging Clear error handling Modular design (retriever, prompt, generator, pipeline) No frontend changes No ingestion logic modification No database schema changes FILE STRUCTURE TO SPECIFY Define responsibilities for the following files: app/rag/ retriever.py # Qdrant similarity search prompt.py # RAG prompt template generator.py # LLM answer generation pipeline.py # End-to-end RAG orchestration app/api/routes/ query.py # FastAPI route for querying ACCEPTANCE CRITERIA User can ask a question via /query System retrieves correct chunks Answer is grounded in stored content No duplicate chunks in context No hallucinated answers Clean and stable API response DELIVERABLE Produce a COMPLETE, CLEAR, and IMPLEMENTABLE SPEC-4 definition including: Responsibilities Data flow Input/output contracts Edge cases Error scenarios DO NOT write actual code. ONLY produce the specification."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query Book Content (Priority: P1)

A user wants to ask a natural language question about the book content and receive an accurate answer based only on the ingested materials. The user accesses the system via a POST API endpoint, submits their question, and receives a response with the answer and source citations.

**Why this priority**: This is the core functionality of the RAG system, allowing users to leverage the ingested book content for knowledge retrieval.

**Independent Test**: Can be fully tested by submitting various questions to the `/query` endpoint and verifying that responses are accurate, grounded in the book content, and include proper citations.

**Acceptance Scenarios**:

1. **Given** a user has entered a natural language question, **When** they submit the question to the `/query` endpoint, **Then** they receive an accurate answer based solely on the ingested book content.
2. **Given** a user enters a question with content available in the ingested books, **When** they submit to the `/query` endpoint, **Then** the system returns an answer with relevant source citations.
3. **Given** a user enters a question with no relevant content in the ingested books, **When** they submit to the `/query` endpoint, **Then** the system returns "Answer not found in provided content".
4. **Given** a user enters a question, **When** they submit to the `/query` endpoint, **Then** the system response takes a reasonable amount of time (< 10 seconds).

---

### User Story 2 - Receive Source Citations (Priority: P2)

When users receive answers from the system, they need to see source citations to verify the accuracy and trustworthiness of the generated answer. The system provides a list of sources showing the book chapters and sections used to generate the response.

**Why this priority**: Provides transparency and accountability for the generated answers, allowing users to verify the source of information and potentially explore the original content.

**Independent Test**: Can be tested by submitting questions and verifying that the response includes a sources array with appropriate book_id, chapter, section, and source_file information.

**Acceptance Scenarios**:

1. **Given** a user submits a question that has relevant content in the book collection, **When** the system generates an answer, **Then** the response includes a sources array listing the relevant books, chapters, and sections.
2. **Given** a question that draws information from multiple book sections, **When** the system responds, **Then** the sources array contains all relevant sections used in the answer generation.

---

### User Story 3 - Handle Various Question Types (Priority: P3)

The system must handle different types of natural language questions including factual queries, conceptual questions, and comparative questions that require pulling information from different parts of the book content.

**Why this priority**: Ensures the system is robust and useful for various types of information-seeking behaviors from users.

**Independent Test**: Can be tested by submitting different types of questions (factual, conceptual, comparative) and verifying that the system appropriately retrieves relevant content and generates accurate answers.

**Acceptance Scenarios**:

1. **Given** a user submits a factual question, **When** the system processes it, **Then** it returns a precise answer based on the relevant book content.
2. **Given** a user submits a conceptual question requiring synthesis of information, **When** the system processes it, **Then** it returns a comprehensive answer drawing from multiple relevant book sections.

### Edge Cases

- What happens when a user submits an extremely long question (thousands of characters)?
- How does the system handle malformed JSON requests?
- What occurs when the vector database is temporarily unavailable?
- How does the system respond when there are connectivity issues with the LLM service?
- What happens when the system receives concurrent high-volume requests?
- How does the system handle questions in different languages?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept a natural language question via a POST endpoint at `/query`
- **FR-002**: System MUST convert the user question into an embedding using the same OpenAI embedding model used in the ingestion process
- **FR-003**: System MUST perform similarity search in Qdrant to retrieve the top-K (default K=5) most relevant vectors from the embedded book content
- **FR-004**: System MUST extract chunk_id, chunk_hash, similarity score, and metadata (book_id, chapter, section, source_file) from the retrieved vectors
- **FR-005**: System MUST fetch the full chunk text from Neon Postgres database using the retrieved chunk_ids
- **FR-006**: System MUST deduplicate retrieved chunks safely to prevent redundant information in the context
- **FR-007**: System MUST construct a deterministic RAG prompt that uses ONLY the retrieved context and forbids hallucination
- **FR-008**: System MUST generate answers using OpenAI Chat Completion with temperature ≤ 0.3 to minimize creative variance
- **FR-009**: System MUST return "Answer not found in provided content" when the retrieved context is insufficient to answer the question 
- **FR-010**: System MUST output clean text without markdown hallucinations in the final answer
- **FR-011**: System MUST return an API response with the answer and a sources array containing source_file, chapter, and section information
- **FR-012**: System MUST preserve original ordering of retrieved chunks where possible to maintain coherence in the context

### Key Entities

- **Question**: A natural language query submitted by a user, consisting of a text string
- **Embedding**: Numerical representation of the question and book content chunks for similarity comparison
- **Retrieved Context**: Aggregated chunks of book content deemed relevant to the user's question
- **Generated Answer**: The final response created by the LLM based on the retrieved context
- **Source Citation**: Metadata pointing to the original location in the books where the answer information was found

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive relevant answers to their questions within 10 seconds of submitting to the `/query` endpoint 95% of the time
- **SC-002**: At least 90% of generated answers are factually accurate when compared to the source book content
- **SC-003**: The system returns source citations with 95% accuracy, correctly identifying the book, chapter, and section where answer information originated
- **SC-004**: When no relevant content exists in the book collection, the system correctly returns "Answer not found in provided content" 98% of the time
- **SC-005**: Users rate the relevance and quality of answers as satisfactory or above 85% of the time in user feedback surveys
- **SC-006**: System has 99.5% uptime under normal usage conditions
- **SC-007**: Less than 2% of generated answers contain hallucinations not grounded in the provided book content