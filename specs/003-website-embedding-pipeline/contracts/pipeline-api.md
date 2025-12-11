# API Contract: Website Embedding Pipeline

## Pipeline Execution API

### POST /pipeline/execute
Initiates the website crawling and embedding pipeline.

**Request**:
```json
{
  "url": "https://hacathoon1-deploy.vercel.app/",
  "collection_name": "reg-embedding",
  "chunk_size": 512,
  "embedding_model": "embed-english-v3.0"
}
```

**Response** (202 Accepted):
```json
{
  "pipeline_id": "pipeline-abc123",
  "status": "initiated",
  "estimated_duration": "60 minutes",
  "created_at": "2023-12-10T23:20:50Z"
}
```

### GET /pipeline/{pipeline_id}
Retrieves the status of a specific pipeline execution.

**Response** (200 OK):
```json
{
  "pipeline_id": "pipeline-abc123",
  "status": "running",
  "pages_crawled": 45,
  "content_extracted": 45,
  "segments_created": 120,
  "embeddings_generated": 120,
  "items_stored": 118,
  "started_at": "2023-12-10T23:20:50Z",
  "progress": 0.65
}
```

### POST /validation/test
Performs a sample query against the vector database to validate successful ingestion.

**Request**:
```json
{
  "query": "What is ROS 2?",
  "collection_name": "reg-embedding",
  "limit": 5
}
```

**Response** (200 OK):
```json
{
  "query": "What is ROS 2?",
  "results": [
    {
      "id": "segment-xyz789",
      "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
      "url": "https://hacathoon1-deploy.vercel.app/docs/ros2/introduction",
      "similarity_score": 0.89
    }
  ]
}
```