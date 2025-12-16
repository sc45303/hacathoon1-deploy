# Data Model: Backend Skeleton & Bootstrap for RAG Chatbot

## Configuration Model

**Entity Name**: AppSettings
- **Description**: Application configuration loaded from environment variables
- **Fields**:
  - `app_name`: str - Name of the application (default: "RAG Chatbot Backend")
  - `app_version`: str - Version of the application (default: "1.0.0")
  - `debug`: bool - Debug mode flag (default: False)
  - `host`: str - Host address for the server (default: "0.0.0.0")
  - `port`: int - Port number for the server (default: 8000)
  - `log_level`: str - Logging level (default: "INFO")
- **Validation Rules**:
  - `port` must be between 1 and 65535
  - `log_level` must be one of "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"
- **Relationships**: None

## Health Response Model

**Entity Name**: HealthResponse
- **Description**: Response model for the health check endpoint
- **Fields**:
  - `status`: str - Health status (always "healthy" for success)
  - `message`: str - Message describing the health state
  - `timestamp`: datetime - ISO 8601 formatted timestamp of the health check
- **Validation Rules**:
  - `status` must be one of "healthy", "degraded", "unhealthy"
  - `message` must be a non-empty string
  - `timestamp` must be in ISO 8601 format
- **Relationships**: None