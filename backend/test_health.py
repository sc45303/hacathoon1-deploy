import asyncio
from datetime import datetime
from app.main import app, health_check
from fastapi.testclient import TestClient

# Create a test client
client = TestClient(app)

def test_health_endpoint():
    """Test the health endpoint"""
    response = client.get("/health")
    
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.json()}")
    
    # Verify status code is 200
    assert response.status_code == 200, f"Expected status 200, got {response.status_code}"
    
    # Verify response structure
    data = response.json()
    assert "status" in data, "Response missing 'status' field"
    assert "message" in data, "Response missing 'message' field"
    assert "timestamp" in data, "Response missing 'timestamp' field"
    
    # Verify field values
    assert data["status"] == "healthy", f"Expected status 'healthy', got {data['status']}"
    assert data["message"] == "Backend is operational", f"Expected message 'Backend is operational', got {data['message']}"
    
    # Verify timestamp format
    try:
        timestamp = datetime.fromisoformat(data["timestamp"].replace("Z", "+00:00"))
        print(f"Timestamp parsed successfully: {timestamp}")
    except ValueError:
        raise AssertionError(f"Could not parse timestamp: {data['timestamp']}")
    
    print("Health endpoint test passed!")

if __name__ == "__main__":
    test_health_endpoint()