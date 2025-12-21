# Quickstart Guide: Backend Skeleton & Bootstrap for RAG Chatbot

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Git (for cloning the repository)

## Setup Instructions

1. **Clone the repository** (if you haven't already):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

3. **Create a virtual environment** (recommended):
   ```bash
   python -m venv venv
   ```

4. **Activate the virtual environment**:
   - On Windows:
     ```bash
     venv\Scripts\activate
     ```
   - On macOS/Linux:
     ```bash
     source venv/bin/activate
     ```

5. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

6. **Create environment file**:
   Copy the example environment file:
   ```bash
   copy .env.example .env  # On Windows
   # or
   cp .env.example .env    # On macOS/Linux
   ```

   You can modify values in `.env` if needed, but defaults should work for initial testing.

7. **Start the development server**:
   ```bash
   uvicorn app.main:app --reload
   ```

## Verification

1. **Check that the server is running**: Open your browser or use curl to visit:
   ```
   http://localhost:8000/health
   ```

2. **Expected response**: You should receive a JSON response similar to:
   ```json
   {
     "status": "healthy",
     "message": "Backend is operational",
     "timestamp": "2025-12-16T10:30:00"
   }
   ```

3. **Check the API documentation**: FastAPI automatically generates interactive API documentation. Visit:
   ```
   http://localhost:8000/docs
   ```

## Project Structure Overview

The backend follows a modular architecture:

```
backend/
├── app/
│   ├── main.py          # Application entry point with health endpoint
│   ├── api/             # API routes (v1)
│   ├── core/            # Configuration and logging
│   ├── services/        # Service layer (future use)
│   ├── db/              # Database connectors (future use)
│   └── models/          # Pydantic models (with HealthResponse model)
├── scripts/             # Utility scripts (future use)
├── requirements.txt     # Python dependencies
├── .env.example         # Environment variable template
└── README.md            # Project documentation
```

## Next Steps

After verifying the backend skeleton is working:

1. Implement database connectors in the `/app/db` directory
2. Add data models in the `/app/models` directory
3. Create service layer functionality in `/app/services`
4. Add API endpoints in `/app/api/v1/`
5. Implement the RAG logic in the service layer