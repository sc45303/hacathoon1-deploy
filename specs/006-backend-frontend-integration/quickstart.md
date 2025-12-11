# Quickstart: Backend-Frontend Integration for RAG Chatbot

## Prerequisites

- Node.js 18+ and npm/yarn for frontend development
- Python 3.11+ for backend development
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

2. **Checkout the integration branch**:
```bash
git checkout 006-backend-frontend-integration
```

3. **Navigate to the website directory for frontend setup**:
```bash
cd website
```

4. **Install frontend dependencies**:
```bash
npm install
# or
yarn install
```

5. **Navigate to the backend directory for backend setup**:
```bash
cd ../backend
```

6. **Install backend dependencies** (if not already installed):
```bash
pip install -e .
```

7. **Set up environment variables**:
Create a `.env` file in the backend directory with the following content:
```
OPENAI_API_KEY=your-openai-api-key
QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
COHERE_API_KEY=your-cohere-api-key
```

8. **Enable CORS in the backend** to allow frontend requests (this will be implemented during the development process)

## Running the Applications

### Running the Frontend (Docusaurus)
```bash
cd website
npm run start
```
This will start the Docusaurus development server on `http://localhost:3000`

### Running the Backend (FastAPI)
```bash
cd backend
uvicorn src.agent.main:app --reload --port 8000
```
This will start the FastAPI server on `http://localhost:8000`

## Testing the Integration

### Manual Testing
1. Start both frontend and backend applications
2. Open the frontend application (should be at http://localhost:3000)
3. Navigate to any page where the chatbot is embedded
4. Enter a question in the chatbot interface
5. Verify that:
   - A loading indicator appears
   - The question is sent to the backend
   - A response is received from the backend
   - The response is displayed in the chat interface
   - If there are sources, they are properly shown

### API Testing
Test the backend API directly using curl:
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the core concepts of the physical AI course?"
  }'
```

## Using the Chatbot Component

The chatbot component can be integrated into Docusaurus pages in several ways:

### Option 1: As an MDX component
In any markdown file:
```md
import Chatbot from '@site/src/components/Chatbot/Chatbot';

<Chatbot />
```

### Option 2: In a layout or theme component
Import and use the Chatbot component in your Docusaurus layout files.

## Configuration

### Frontend Configuration
The chatbot component may accept configuration props:
- `backendUrl`: The URL of the backend API (defaults to /api when relative)
- `initialMessages`: Optional array of initial messages to display
- `placeholderText`: Placeholder text for the input field
- `title`: Title for the chat interface

### Backend Configuration
The backend API endpoint (POST /ask) supports:
- `session_id`: To maintain conversation context
- `user_id`: To track user interactions

## Troubleshooting

- If the frontend can't connect to the backend, check if CORS is properly configured
- If you get "Authentication Error", check that your API keys in `.env` are correct
- If the chatbot doesn't appear in the page, verify that the component is properly imported and used
- For development, you might need to configure proxy settings if running both servers separately

## Next Steps

1. Implement and test the React chatbot component with loading and error states
2. Update the backend to properly handle CORS requests from localhost:3000
3. Add the chatbot component to relevant pages in the Docusaurus site
4. Implement proper error handling and user feedback mechanisms
5. Consider implementing rate limiting and monitoring for the API endpoints