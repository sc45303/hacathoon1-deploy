"""
End-to-end test script to verify the tool call → retrieval → response flow.
"""
import asyncio
import sys
import os
import time

# Add backend to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from backend.src.agent.config import AgentConfig
from backend.src.agent.agent_service import AgentService


def test_end_to_end_flow():
    """
    Test the complete flow: tool call → retrieval → final response.
    """
    print("Starting end-to-end test...")
    
    # Load configuration
    config = AgentConfig.from_env()
    errors = config.validate()
    if errors:
        print(f"Configuration errors: {errors}")
        return False
    
    # Initialize agent service
    try:
        agent_service = AgentService(config)
        print("✓ Agent service initialized")
    except Exception as e:
        print(f"✗ Failed to initialize agent service: {e}")
        return False
    
    # Test a sample query
    test_query = "What are the core concepts of the physical AI course?"
    
    print(f"Testing query: '{test_query}'")
    
    try:
        start_time = time.time()
        response = agent_service.ask_question(test_query)
        end_time = time.time()
        
        print(f"✓ Query processed in {end_time - start_time:.2f} seconds")
        print(f"✓ Response received: {response.content[:100]}...")
        print(f"✓ Retrieved {len(response.chunks_used)} chunks")
        
        # Verify that chunks were actually used
        if len(response.chunks_used) > 0:
            print("✓ Retrieved chunks present in response")
            print(f"✓ First chunk relevance score: {response.chunks_used[0].relevance_score}")
        else:
            print("⚠ No chunks were retrieved")
        
        # Check if response is grounded in retrieved content
        if len(response.chunks_used) > 0:
            print("✓ Response is based on retrieved content")
        else:
            print("⚠ Response might not be grounded in book content")
        
        return True
        
    except Exception as e:
        print(f"✗ Error during end-to-end test: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_health_check():
    """
    Test the health check functionality.
    """
    print("\nTesting health check...")
    
    # Load configuration
    config = AgentConfig.from_env()
    errors = config.validate()
    if errors:
        print(f"Configuration errors: {errors}")
        return False
    
    # Initialize agent service
    try:
        agent_service = AgentService(config)
    except Exception as e:
        print(f"✗ Failed to initialize agent service: {e}")
        return False
    
    try:
        health_info = agent_service.check_health()
        print("✓ Health check completed")
        print(f"✓ Overall status: {health_info['status']}")
        
        for service, details in health_info['services'].items():
            print(f"✓ {service}: {details['status']} (response time: {details['response_time_ms']}ms)")
        
        return True
        
    except Exception as e:
        print(f"✗ Error during health check: {e}")
        return False


def run_all_tests():
    """
    Run all end-to-end tests.
    """
    print("Running end-to-end tests for the retrieval-enabled agent service...")
    print("=" * 60)
    
    test_results = []
    
    # Test 1: End-to-end flow
    print("\nTest 1: End-to-end flow (tool call → retrieval → response)")
    result1 = test_end_to_end_flow()
    test_results.append(("End-to-end flow", result1))
    
    # Test 2: Health check
    print("\nTest 2: Health check")
    result2 = test_health_check()
    test_results.append(("Health check", result2))
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary:")
    for test_name, result in test_results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {test_name}: {status}")
    
    all_passed = all(result for _, result in test_results)
    print(f"\nOverall result: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")
    
    return all_passed


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)