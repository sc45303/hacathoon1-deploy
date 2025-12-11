# Quickstart: Website Embedding Pipeline

## Setup

1. **Prerequisites**:
   - Python 3.11+
   - UV package manager
   - Cohere API key
   - Qdrant Cloud account and API key

2. **Clone and Navigate**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   cd backend
   ```

3. **Install Dependencies with UV**:
   ```bash
   uv sync
   ```

4. **Environment Setup**:
   ```bash
   cp .env.example .env
   # Edit .env with your Cohere and Qdrant credentials
   ```

## Configuration

Set up your environment variables in `.env`:

```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Usage

Run the complete pipeline:

```bash
cd backend
uv run python -m src.website_pipeline.main --url "https://hacathoon1-deploy.vercel.app/"
```

## Validation

After the pipeline completes, you can validate the results:

1. Check the console output for processing statistics
2. Verify the 'reg-embedding' collection exists in your Qdrant instance
3. Test sample retrieval with the validation script:
   ```bash
   uv run python -m src.website_pipeline.validation
   ```

## Customization

- To change the target URL: modify the `--url` parameter
- To change the Qdrant collection name: modify the `--collection-name` parameter
- To change chunk size: modify the `--max-tokens` parameter