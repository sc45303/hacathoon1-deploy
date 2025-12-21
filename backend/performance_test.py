"""
Performance testing script for the agent endpoint.

This script tests that the agent endpoint responds within the required
performance threshold of <5 seconds for all query types.
"""
import time
import requests
import asyncio
import aiohttp
from typing import Dict, Any


BASE_URL = "http://localhost:8000"


def test_response_time_sync(query_data: Dict[str, Any], endpoint: str = "/agent/query") -> float:
    """Test response time for a single synchronous request."""
    start_time = time.time()
    
    try:
        response = requests.post(f"{BASE_URL}{endpoint}", json=query_data, timeout=10)
        response_time = time.time() - start_time
        
        print(f"Status: {response.status_code}, Time: {response_time:.2f}s")
        return response_time
    except Exception as e:
        print(f"Error: {e}")
        return float('inf')


async def test_response_time_async(session: aiohttp.ClientSession, query_data: Dict[str, Any], endpoint: str = "/agent/query") -> float:
    """Test response time for a single asynchronous request."""
    start_time = time.time()
    
    try:
        async with session.post(f"{BASE_URL}{endpoint}", json=query_data) as response:
            await response.text()  # Read the response
            response_time = time.time() - start_time
            
            print(f"Status: {response.status}, Time: {response_time:.2f}s")
            return response_time
    except Exception as e:
        print(f"Error: {e}")
        return float('inf')


async def performance_test():
    """Run performance tests on the agent endpoint."""
    print("Starting performance tests for agent endpoint...")
    
    # Test cases for different modes
    test_cases = [
        {
            "name": "Book RAG Mode",
            "data": {"question": "What are the main themes in this book?"},
            "expected_mode": "book"
        },
        {
            "name": "Selected Text Mode",
            "data": {
                "question": "Can you explain this concept?",
                "selected_text": "The main themes of this book include the struggle between good and evil, the power of friendship, and the importance of perseverance."
            },
            "expected_mode": "selected_text"
        },
        {
            "name": "General Knowledge Mode",
            "data": {
                "question": "What is the capital of France?",
                "mode": "general"
            },
            "expected_mode": "general"
        }
    ]
    
    # Performance thresholds
    MAX_RESPONSE_TIME = 5.0  # seconds
    
    async with aiohttp.ClientSession() as session:
        for test_case in test_cases:
            print(f"\nTesting {test_case['name']}...")
            
            # Run multiple requests to get average performance
            response_times = []
            for i in range(3):  # Run 3 times to get average
                print(f"  Request {i+1}/3...")
                response_time = await test_response_time_async(session, test_case['data'])
                response_times.append(response_time)
            
            avg_response_time = sum(response_times) / len(response_times)
            max_response_time = max(response_times)
            
            print(f"  Average response time: {avg_response_time:.2f}s")
            print(f"  Max response time: {max_response_time:.2f}s")
            
            if max_response_time > MAX_RESPONSE_TIME:
                print(f"  ❌ FAIL: Max response time ({max_response_time:.2f}s) exceeds threshold ({MAX_RESPONSE_TIME}s)")
            else:
                print(f"  ✅ PASS: Response time within threshold")
    
    print("\nPerformance testing completed.")


if __name__ == "__main__":
    # Note: This requires the backend server to be running
    print("Please ensure the backend server is running on http://localhost:8000 before running this test.")
    print("Start the server with: uvicorn app.main:app --reload")
    
    asyncio.run(performance_test())