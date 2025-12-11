"""
Validation Script

This script performs sample queries against the stored embeddings to validate successful ingestion.
"""
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import cohere


# Load environment variables
load_dotenv()

# Initialize Cohere client
co = cohere.Client(os.getenv("COHERE_API_KEY"))

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10
)


def validate_ingestion(collection_name: str = "reg-embedding", query: str = "What is this book about?"):
    """
    Perform a sample query against the vector database to validate successful ingestion.
    
    Args:
        collection_name: Name of the collection to query
        query: The query text to search for similar content
        
    Returns:
        List of results with content and similarity scores
    """
    try:
        # Generate embedding for the query
        response = co.embed(
            texts=[query],
            model='embed-english-v3.0',
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]
        
        # Search in Qdrant
        search_result = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=5,
            with_payload=True
        )
        
        results = []
        for hit in search_result:
            results.append({
                "id": hit.id,
                "content": hit.payload.get("text", "")[:200] + "...",  # Truncate for display
                "url": hit.payload.get("url", ""),
                "similarity_score": hit.score
            })
        
        return results
    except Exception as e:
        print(f"Validation failed: {str(e)}")
        return []


def main():
    """Run validation to test sample retrieval and ensure ingestion works."""
    print("Validating ingestion by performing sample queries...")
    
    # Test query
    test_query = "What is this book about?"
    
    results = validate_ingestion("reg-embedding", test_query)
    
    if results:
        print(f"Found {len(results)} similar content items:")
        for i, result in enumerate(results, 1):
            print(f"{i}. URL: {result['url']}")
            print(f"   Content: {result['content']}")
            print(f"   Similarity: {result['similarity_score']:.3f}")
            print()
    else:
        print("No results found. Ingestion may not have completed successfully.")


if __name__ == "__main__":
    main()