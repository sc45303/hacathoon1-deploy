# Data Model: Retrieval Pipeline Validation

## Core Entities

### ValidationQuery
- **id**: string - Unique identifier for the validation query
- **text**: string - The query text to be validated
- **type**: enum(factual, conceptual, section-specific) - Type of query
- **expected_section**: string - The expected section or content that should be retrieved
- **metadata**: dict - Additional metadata about the query

### RetrievedChunk
- **id**: string - Unique identifier for the chunk
- **content**: string - The text content of the retrieved chunk
- **url**: string - The source URL of the content
- **position**: integer - The position of the chunk in the original document
- **relevance_score**: float - Similarity score from the retrieval process
- **source_metadata**: dict - Metadata from when the chunk was originally stored

### ValidationResult
- **query_id**: string - Reference to the validation query
- **retrieved_chunks**: list[RetrievedChunk] - The top retrieved chunks for the query
- **accuracy_score**: float - Overall accuracy score for this query
- **correctness**: boolean - Whether the results are considered accurate
- **comparison_details**: dict - Detailed comparison between expected and retrieved content
- **failure_reason**: string? - Reason for failure if results were inaccurate

### ValidationReport
- **id**: string - Unique identifier for the report
- **date**: datetime - When the validation was run
- **total_queries**: integer - Total number of queries tested
- **successful_queries**: integer - Number of successful queries
- **accuracy_percentage**: float - Overall accuracy percentage
- **detailed_results**: list[ValidationResult] - Individual results for each query
- **failure_analysis**: dict - Summary of common failure patterns
- **query_type_breakdown**: dict - Accuracy by query type (factual, conceptual, section-specific)

## Relationships

- A `ValidationQuery` generates one `ValidationResult`
- A `ValidationResult` contains multiple `RetrievedChunk` objects
- Multiple `ValidationResult` objects contribute to one `ValidationReport`

## Validation Rules

### From Functional Requirements

- **FR-001**: System MUST connect to the existing Qdrant collection to fetch stored embeddings
  - `RetrievedChunk` must have valid references to the Qdrant collection

- **FR-002**: System MUST generate embeddings for query text using Cohere models for similarity search
  - `ValidationQuery` text must be transformed to embeddings compatible with the stored vectors

- **FR-003**: System MUST perform similarity search in Qdrant and return top 3-5 most relevant chunks
  - `ValidationResult` must contain between 3-5 `RetrievedChunk` objects

- **FR-004**: System MUST compare retrieved chunks with original book text to verify correctness
  - `ValidationResult` must include `comparison_details` comparing retrieved vs expected content

- **FR-005**: System MUST validate retrieval accuracy across multiple query types: factual, conceptual, section-specific
  - `ValidationReport` must include breakdown by query `type`

- **FR-006**: System MUST generate validation reports summarizing accuracy metrics and failure cases
  - `ValidationReport` must include `total_queries`, `successful_queries`, `accuracy_percentage`, and `failure_analysis`

### Success Criteria Compliance

- **SC-001**: At least 90% of test queries return relevant and accurate results
  - `accuracy_percentage` in `ValidationReport` should be >= 0.9

- **SC-004**: Validation reports clearly show the top 3–5 retrieved chunks per test query with relevance scores
  - Each `ValidationResult` must list 3-5 `RetrievedChunk` objects with `relevance_score`