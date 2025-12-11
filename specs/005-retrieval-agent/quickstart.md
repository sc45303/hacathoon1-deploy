# Quickstart: Retrieval-Enabled Agent Service

## Prerequisites

- Python 3.11 or higher
- Access to OpenAI API (API key)
- Access to Qdrant Cloud instance with book embeddings
- Cohere API key
- Git

## Setup

1. **Clone the repository** (if not already done):
```bash
git clone <repository-url>
cd <repository-name>
```

2. **Checkout the agent branch**:
```bash
git checkout 005-retrieval-agent
```

3. **Navigate to the backend directory**:
```bash
cd backend
```

4. **Install dependencies**:
```bash
pip install fastapi openai qdrant-client cohere pydantic python-dotenv uvicorn
```

5. **Set up environment variables**:
Create a `.env` file in the backend directory with the following content:
```
OPENAI_API_KEY=your-openai-api-key
QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
COHERE_API_KEY=your-cohere-api-key
```

## Running the Service

### Start the Server
```bash
cd backend
uvicorn src.agent.main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`

### Test the Health Endpoint
```bash
curl http://localhost:8000/health
```

### Test the Ask Endpoint
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the core modules of the Physical AI course?"
  }'
```

## Understanding the Architecture

The agent service consists of several key components:

1. **FastAPI Application** - Main web server entry point (`main.py`)
2. **Agent Service** - Core OpenAI Agent implementation (`agent_service.py`)
3. **Retrieval Tool** - Custom tool connecting to Qdrant (`retrieval_tool.py`)
4. **Data Models** - Pydantic models for request/response validation (`models.py`)
5. **Configuration** - Settings management (`config.py`)

## API Endpoints

### `/ask` (POST)
Submit questions about the book content and receive sourced answers.
- Request: `{"query": "your question here"}`
- Response: Answer with cited sources from the book

### `/health` (GET)
Check the system status and dependency health.
- Returns health status of OpenAI, Qdrant, and Cohere services

### `/agent/run` (POST)
Programmatically execute agent runs with custom instructions.
- Request: `{"instructions": "what to do", "input_data": {...}}`
- Response: Agent output with tool call information

## Troubleshooting

- If you get "Authentication Error", check that your API keys in `.env` are correct
- If queries return no results, verify that your Qdrant collection has the book embeddings
- If the service doesn't start, make sure all dependencies are installed
- For performance issues, check that your network connection to the APIs is stable

## Next Steps

1. Integrate the agent service with your frontend application
2. Implement client-side error handling and retry logic
3. Set up monitoring and logging for production deployment
4. Consider implementing caching for frequently asked questions