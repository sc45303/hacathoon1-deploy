# Data Model: RAG Query System

## Core Entities

### Question
- **Description**: A natural language query submitted by a user
- **Fields**:
  - text (string): The actual question content submitted by the user
- **Validation**:
  - Must not be empty
  - Should have reasonable length limits (e.g., max 1000 characters)
- **Relationships**: Input to the RAG pipeline

### Embedding
- **Description**: Numerical representation of the question and book content chunks for similarity comparison
- **Fields**:
  - vector (list of floats): The embedding vector representation
  - text_content (string): The original text that was embedded
  - model_used (string): The model that generated the embedding (for consistency)
- **Validation**:
  - Vector must be properly normalized
  - model_used must match the ingestion model
- **Relationships**: Used for similarity search between question and book chunks

### RetrievedContext
- **Description**: Aggregated chunks of book content deemed relevant to the user's question
- **Fields**:
  - chunks (list of ChunkReference): The retrieved book content chunks
  - relevance_scores (list of floats): Similarity scores for each chunk
  - metadata (list of dict): Additional metadata about each chunk
- **Validation**:
  - Chunks must be deduplicated
  - Relevance scores must be in descending order (most relevant first)
- **Relationships**: Output from the retrieval process, input to the generation process

### ChunkReference
- **Description**: Reference to a specific chunk from the book content database
- **Fields**:
  - chunk_id (string): Unique identifier for the chunk
  - chunk_hash (string): Hash value for integrity verification
  - content (string): The actual text content of the chunk
  - book_id (string): ID of the book this chunk belongs to
  - chapter (string): Chapter title/identifier
  - section (string): Section title/identifier
  - source_file (string): Original file name
  - similarity_score (float): How similar this chunk is to the question
- **Validation**:
  - chunk_id must be unique
  - content must not be empty
- **Relationships**: Part of RetrievedContext, links to book content in Neon Postgres

### GeneratedAnswer
- **Description**: The final response created by the LLM based on the retrieved context
- **Fields**:
  - text (string): The generated answer text
  - sources (list of SourceCitation): References to the book sections used
  - confidence_score (float): Estimated confidence in the answer (optional)
- **Validation**:
  - Must only contain information from the provided context
  - Must be clean text without markdown hallucinations
- **Relationships**: Output from the generation process, response to the original question

### SourceCitation
- **Description**: Metadata pointing to the original location in the books where the answer information was found
- **Fields**:
  - source_file (string): Original file name of the source
  - chapter (string): Chapter title/identifier
  - section (string): Section title/identifier
- **Validation**:
  - All fields must be filled
- **Relationships**: Part of the GeneratedAnswer, references specific locations in the book content

## API Request/Response Objects

### QueryRequest
- **Description**: The input to the `/query` endpoint
- **Fields**:
  - question (string): The natural language question from the user
- **Validation**:
  - question must not be empty
  - question must be a string
- **Example**:
```json
{
  "question": "What are the key principles of quantum computing?"
}
```

### QueryResponse
- **Description**: The output from the `/query` endpoint
- **Fields**:
  - answer (string): The generated answer to the question
  - sources (list of SourceCitation): List of sources used in the answer
- **Validation**:
  - answer must not be empty (unless returning "Answer not found in provided content")
  - sources list may be empty if no relevant content was found
- **Example**:
```json
{
  "answer": "Quantum computing relies on the principles of superposition and entanglement...",
  "sources": [
    {
      "source_file": "quantum-physics-book.md",
      "chapter": "Introduction to Quantum Computing",
      "section": "Basic Principles"
    }
  ]
}
```