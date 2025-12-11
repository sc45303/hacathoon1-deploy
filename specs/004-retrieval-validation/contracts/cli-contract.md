# CLI Contract: Retrieval Pipeline Validation Tool

## Command Line Interface

### Main Validation Command
```bash
python -m website_pipeline.validation_runner [options]
```

### Options
- `--config-path` [string]: Path to configuration file (default: ".env")
- `--collection-name` [string]: Qdrant collection name to validate (default: "reg-embedding")
- `--test-queries-file` [string]: Path to file containing test queries (default: "validation/test_queries.json")
- `--output-path` [string]: Path to save validation report (default: "validation_report.json")
- `--output-format` [string]: Output format (json, text) (default: "json")
- `--threshold` [float]: Minimum accuracy threshold (default: 0.9)
- `--top-k` [int]: Number of top results to retrieve (default: 5)
- `--query-types` [string]: Comma-separated list of query types to test (default: "factual,conceptual,section-specific")

## Input Format

### Test Queries JSON Schema
```json
{
  "queries": [
    {
      "id": "string",
      "text": "string",
      "type": "enum(factual, conceptual, section-specific)",
      "expected_section": "string",
      "metadata": {}
    }
  ]
}
```

## Output Format

### Validation Report JSON Schema
```json
{
  "id": "string",
  "date": "datetime",
  "total_queries": "integer",
  "successful_queries": "integer",
  "accuracy_percentage": "float",
  "detailed_results": [
    {
      "query_id": "string",
      "retrieved_chunks": [
        {
          "id": "string",
          "content": "string",
          "url": "string",
          "position": "integer",
          "relevance_score": "float",
          "source_metadata": {}
        }
      ],
      "accuracy_score": "float",
      "correctness": "boolean",
      "comparison_details": {},
      "failure_reason": "string?"
    }
  ],
  "failure_analysis": {},
  "query_type_breakdown": {
    "factual": {"accuracy": "float", "count": "integer"},
    "conceptual": {"accuracy": "float", "count": "integer"},
    "section-specific": {"accuracy": "float", "count": "integer"}
  }
}
```

## Exit Codes
- `0`: Validation completed successfully with accuracy >= threshold
- `1`: Validation completed but accuracy < threshold
- `2`: Validation failed due to runtime error
- `3`: Invalid input parameters

## Configuration Requirements

The tool expects the following environment variables to be set:
- `QDRANT_URL`: URL to the Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud access
- `COHERE_API_KEY`: API key for Cohere embedding service

## Dependencies
- Python 3.11+
- qdrant-client
- cohere
- python-dotenv

## Performance Expectations
- Process 100 validation queries in under 5 minutes
- Memory usage under 50MB during operation
- Each query response time under 5 seconds