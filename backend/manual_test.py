"""
Manual API testing script for the agent endpoint.

This script tests the agent endpoint with examples from quickstart.md
to verify that it works as expected.
"""
import requests
import json
from typing import Dict, Any


BASE_URL = "http://localhost:8000"


def test_api_endpoint(endpoint: str, data: Dict[str, Any], test_name: str):
    """Test a single API endpoint with provided data."""
    print(f"\n--- Testing: {test_name} ---")
    print(f"Endpoint: {endpoint}")
    print(f"Data: {json.dumps(data, indent=2)}")
    
    try:
        response = requests.post(f"{BASE_URL}{endpoint}", json=data, timeout=10)
        print(f"Status Code: {response.status_code}")
        
        if response.status_code == 200:
            response_data = response.json()
            print(f"Response: {json.dumps(response_data, indent=2)}")
            
            # Validate response structure
            required_fields = ["answer", "sources", "mode"]
            for field in required_fields:
                if field not in response_data:
                    print(f"❌ FAIL: Missing field '{field}' in response")
                    return False
            
            print("✅ PASS: Response has correct structure")
            return True
        else:
            print(f"❌ FAIL: Status code {response.status_code}")
            print(f"Response: {response.text}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("❌ FAIL: Cannot connect to server. Is the backend running on http://localhost:8000?")
        return False
    except Exception as e:
        print(f"❌ FAIL: Error occurred: {e}")
        return False


def run_manual_tests():
    """Run manual API tests based on examples from quickstart.md."""
    print("Starting manual API tests based on quickstart.md examples...")
    print("Make sure the backend server is running on http://localhost:8000")
    
    all_tests_passed = True
    
    # Test 1: Book RAG Query (Default Mode)
    test_data_1 = {
        "question": "What are the main themes in this book?"
    }
    result_1 = test_api_endpoint("/agent/query", test_data_1, "Book RAG Query (Default Mode)")
    all_tests_passed = all_tests_passed and result_1
    
    # Test 2: Selected Text Query
    test_data_2 = {
        "question": "Can you explain this concept?",
        "selected_text": "The main themes of this book include the struggle between good and evil, the power of friendship, and the importance of perseverance."
    }
    result_2 = test_api_endpoint("/agent/query", test_data_2, "Selected Text Query")
    all_tests_passed = all_tests_passed and result_2
    
    # Test 3: General Knowledge Query
    test_data_3 = {
        "question": "What is the capital of France?",
        "mode": "general"
    }
    result_3 = test_api_endpoint("/agent/query", test_data_3, "General Knowledge Query")
    all_tests_passed = all_tests_passed and result_3
    
    # Additional test: Verify that the original /query endpoint still works
    test_data_4 = {
        "question": "What are the main themes in this book?"
    }
    result_4 = test_api_endpoint("/query", test_data_4, "Original /query endpoint (should still work)")
    all_tests_passed = all_tests_passed and result_4
    
    print(f"\n--- Test Summary ---")
    if all_tests_passed:
        print("✅ All manual API tests PASSED!")
        print("The agent endpoint is working correctly with all query types.")
    else:
        print("❌ Some manual API tests FAILED!")
        print("Please check the backend implementation.")
    
    return all_tests_passed


if __name__ == "__main__":
    run_manual_tests()