# Retrieval Pipeline Validation Tool

This tool validates the retrieval pipeline by connecting to the Qdrant vector database, 
performing similarity searches using Cohere embeddings, and comparing results with 
expected content to verify accuracy.

## Purpose

The validation tool ensures that the RAG (Retrieval-Augmented Generation) system 
is functioning correctly by:
- Querying the vector database with test queries
- Comparing retrieved chunks with expected content
- Generating reports on accuracy and failure cases
- Confirming the system meets the 90%+ accuracy threshold

## Prerequisites

- Python 3.11+
- Valid Qdrant Cloud credentials
- Valid Cohere API key
- Embeddings already stored in Qdrant collection

## Configuration

Create a `.env` file in the backend directory with the following variables:

```bash
QDRANT_URL=https://your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key
COHERE_API_KEY=your-cohere-api-key
```

## Usage

### Basic Validation

```bash
cd backend
python -m website_pipeline.validation_runner
```

### Custom Parameters

```bash
python -m website_pipeline.validation_runner \
    --collection-name "reg-embedding" \
    --test-queries-file "validation/test_queries.json" \
    --output-path "results/validation_report.json" \
    --threshold 0.9 \
    --top-k 5 \
    --output-format "json"
```

### Select Query Types

```bash
python -m website_pipeline.validation_runner \
    --query-types "factual,conceptual"
```

## Command Line Options

- `--config-path`: Path to configuration file (default: ".env")
- `--collection-name`: Qdrant collection name to validate (default: "reg-embedding")
- `--test-queries-file`: Path to file containing test queries (default: "validation/test_queries.json")
- `--output-path`: Path to save validation report (default: "validation_report.json")
- `--output-format`: Output format (json, text) (default: "json")
- `--threshold`: Minimum accuracy threshold (default: 0.9)
- `--top-k`: Number of top results to retrieve (default: 5)
- `--query-types`: Comma-separated list of query types to test (default: "factual,conceptual,section-specific")

## Exit Codes

- `0`: Validation completed successfully with accuracy >= threshold
- `1`: Validation completed but accuracy < threshold
- `2`: Validation failed due to runtime error
- `3`: Invalid input parameters

## Output

The tool generates two types of reports:

### JSON Report
Detailed validation results in structured JSON format including:
- Individual query results with retrieved chunks
- Accuracy scores for each query
- Failure analysis
- Query type breakdown statistics

### Text Report
Human-readable summary including:
- Overall statistics
- Accuracy by query type
- Top failure reasons
- Individual query results

## Test Queries Format

Test queries are defined in JSON format:

```json
{
  "queries": [
    {
      "id": "factual_001",
      "text": "What is the primary goal of the course?",
      "type": "factual",
      "expected_section": "Course objectives and goals",
      "metadata": {
        "difficulty": "easy",
        "domain": "course-introduction"
      }
    }
  ]
}
```

## Architecture

The validation system consists of several modules:

- `validation_runner.py`: Main entry point and orchestration
- `retrieval_validator.py`: Core validation logic (Qdrant/Cohere integration)
- `result_comparator.py`: Content comparison and accuracy scoring
- `report_generator.py`: Report generation and analysis
- `validation_data_models.py`: Data structures for validation results
- `validation_config.py`: Configuration management

## Testing

Unit and integration tests are available in the `backend/tests/test_validation.py` file.
Run them using:

```bash
cd backend
pytest tests/test_validation.py
```

## Troubleshooting

- If you get authentication errors, verify your API keys in `.env`
- If queries return no results, confirm embeddings are properly stored in Qdrant
- For poor accuracy, consider re-evaluating the embedding process or query formulation