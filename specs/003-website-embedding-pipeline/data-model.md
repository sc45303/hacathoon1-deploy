# Data Model: Website Embedding Pipeline

## Entities

### WebpageContent
- **id**: String (unique identifier for the content)
- **url**: String (the source URL of the webpage)
- **title**: String (the title of the webpage)
- **content**: String (the clean extracted text from the webpage)
- **created_at**: DateTime (timestamp when the content was extracted)
- **metadata**: Object (additional metadata like headings, sections, etc.)

### TextSegment
- **id**: String (unique identifier for the segment)
- **content**: String (the chunked text content)
- **original_id**: String (reference to the original WebpageContent)
- **position**: Integer (the order of this segment in the original content)
- **token_count**: Integer (number of tokens in the segment)
- **metadata**: Object (additional metadata including source URL)

### VectorRepresentation
- **id**: String (unique identifier for the vector)
- **segment_id**: String (reference to the TextSegment)
- **vector**: Array<Float> (the embedding values)
- **model_used**: String (the model used to generate the embedding)
- **created_at**: DateTime (timestamp when the embedding was generated)
- **metadata**: Object (additional metadata including source URL)

### PipelineStatus
- **id**: String (unique identifier for the pipeline run)
- **status**: String (e.g., "running", "completed", "failed")
- **pages_crawled**: Integer (number of pages successfully crawled)
- **content_extracted**: Integer (number of content items extracted)
- **segments_created**: Integer (number of text segments created)
- **embeddings_generated**: Integer (number of embeddings generated)
- **items_stored**: Integer (number of items stored in Qdrant)
- **started_at**: DateTime (timestamp when the pipeline started)
- **completed_at**: DateTime (timestamp when the pipeline completed)