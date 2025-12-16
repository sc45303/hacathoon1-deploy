# OpenAPI Contract: Backend Skeleton & Bootstrap for RAG Chatbot

## Health Endpoint

### GET /health

**Summary**: Health check endpoint to verify backend is operational

**Description**: Returns a simple health status response to verify that the backend application is running correctly.

**Parameters**: None

**Responses**:
- **200**: Successful response with health status
  - Content-Type: application/json
  - Schema: HealthResponse
    - status: string (e.g., "healthy")
    - message: string (e.g., "Backend is operational")
    - timestamp: string (ISO 8601 datetime format)

**Example Response**:
```json
{
  "status": "healthy",
  "message": "Backend is operational",
  "timestamp": "2025-12-16T10:30:00Z"
}
```

**Authentication**: None required

**Tags**: [health, monitoring]

**Implementation Notes**: This endpoint should return quickly (under 1 second) and not perform any complex operations that might cause delays or failures.