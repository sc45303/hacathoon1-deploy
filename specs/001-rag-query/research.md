# Research: RAG Query System

## Unknowns Resolved

### 1. Qdrant Vector Similarity Search Parameters
- **Decision**: Use cosine similarity with top-K retrieval (K=5) as specified
- **Rationale**: Feature spec explicitly states to retrieve top-K most relevant vectors with default K=5
- **Alternatives considered**: Euclidean distance, Manhattan distance - but spec mandates the approach to use

### 2. OpenAI Embedding Model for Query Processing
- **Decision**: Use the same OpenAI embedding model that was used in the ingestion process
- **Rationale**: Feature spec explicitly requires using the SAME OpenAI embedding model used in ingestion
- **Alternatives considered**: Different embedding models - but this would create inconsistency with ingested vectors

### 3. Error Handling Strategy for Service Dependencies
- **Decision**: Implement graceful degradation with appropriate fallback messages
- **Rationale**: The system must handle cases when Qdrant, Neon, or OpenAI services are unavailable
- **Alternatives considered**: Complete failure vs. graceful degradation - chose graceful degradation for better user experience

### 4. Chunk Deduplication Strategy
- **Decision**: Implement deduplication at the chunk_id level to prevent redundant information
- **Rationale**: Feature spec specifically requires safe deduplication of chunks to prevent redundant context
- **Alternatives considered**: Content-based deduplication vs. ID-based deduplication - ID-based is more efficient and reliable

## Technology Best Practices Researched

### 1. FastAPI Async Implementation Patterns
- **Decision**: Implement fully async architecture throughout the RAG pipeline
- **Rationale**: Backend stack is FastAPI with async Python, and non-functional requirement specifies fully async with no blocking calls
- **Implementation**: Use async/await for all I/O operations including DB queries and external API calls

### 2. OpenAI Chat Completion Best Practices
- **Decision**: Use low temperature (≤ 0.3) as specified and implement proper system/user message formatting
- **Rationale**: Feature spec explicitly requires temperature ≤ 0.3 to minimize creative variance
- **Safety measures**: Implement prompt injection protection and content filtering

### 3. RAG Prompt Engineering
- **Decision**: Create a deterministic prompt template that strictly enforces context-only responses
- **Rationale**: Feature spec requires a prompt that forbids hallucination and returns "Answer not found in provided content" if context is insufficient
- **Best practice**: Include clear instructions and examples in the system prompt to guide the LLM behavior

### 4. Source Citation Implementation
- **Decision**: Include source metadata (book_id, chapter, section, source_file) in the response as specified
- **Rationale**: Feature spec requires returning source information to provide transparency and accountability
- **Format**: Structure as an array of source objects with the specified fields