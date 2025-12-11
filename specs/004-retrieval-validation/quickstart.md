# Quickstart: Retrieval Pipeline Validation

## Prerequisites

- Python 3.11 or higher
- Access to the Qdrant Cloud instance with book embeddings
- Cohere API key
- Git

## Setup

1. **Clone the repository** (if not already done):
```bash
git clone <repository-url>
cd <repository-name>
```

2. **Checkout the validation branch**:
```bash
git checkout 004-retrieval-validation
```

3. **Navigate to the backend directory**:
```bash
cd backend
```

4. **Install dependencies** (if not already installed):
```bash
pip install -e .
```

5. **Set up environment variables**:
Create a `.env` file in the backend directory with the following content:
```
QDRANT_URL=https://your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key
COHERE_API_KEY=your-cohere-api-key
```

## Running the Validation

### Basic Validation
```bash
python -m website_pipeline.validation_runner
```

### Validation with Custom Parameters
```bash
python -m website_pipeline.validation_runner \
    --collection-name "reg-embedding" \
    --test-queries-file "validation/test_queries.json" \
    --output-path "results/validation_report.json" \
    --threshold 0.9 \
    --top-k 5
```

### Validation for Specific Query Types
```bash
python -m website_pipeline.validation_runner \
    --query-types "factual,conceptual"
```

## Understanding the Output

After running the validation, you'll get:

1. **Console Output**: Summary statistics during the validation process
2. **JSON Report**: Detailed results in the specified output path
3. **Validation Report**: Shows:
   - Overall accuracy percentage
   - Results for each query
   - Top retrieved chunks with relevance scores
   - Failure analysis

## Sample Validation Report

```json
{
  "accuracy_percentage": 0.92,
  "total_queries": 50,
  "successful_queries": 46,
  "query_type_breakdown": {
    "factual": {"accuracy": 0.94, "count": 20},
    "conceptual": {"accuracy": 0.89, "count": 15},
    "section-specific": {"accuracy": 0.93, "count": 15}
  }
}
```

## Troubleshooting

- If you get `ModuleNotFoundError`, ensure you've installed the package with `pip install -e .`
- If you get authentication errors, check that your API keys in `.env` are correct
- If validation takes too long, verify your network connection to Qdrant and Cohere services

## Next Steps

1. Examine the validation report to identify areas where the retrieval pipeline could be improved
2. Adjust your test queries based on the failure analysis
3. Consider re-running validation after making improvements to the embedding or retrieval process