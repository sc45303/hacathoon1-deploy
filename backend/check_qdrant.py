import asyncio
from app.db.qdrant import QdrantDB

async def test_qdrant():
    db = QdrantDB()
    try:
        print("Connecting to Qdrant...")
        is_connected = await db.validate_connection()
        print(f"Qdrant connection status: {is_connected}")
        
        if is_connected:
            # Check collection status
            health = await db.health_check()
            print(f"Qdrant health: {health}")
            
            # Count points in collection
            client = await db._get_client()
            collection_info = await client.get_collection(db._collection_name)
            print(f"Collection '{db._collection_name}' vector count: {collection_info.points_count}")
            
            # Try to search with a simple query to see if embeddings exist
            # For this, we need a dummy embedding; let's just get the first few points
            records = await client.retrieve(
                collection_name=db._collection_name,
                ids=["00000000-0000-0000-0000-000000000000"] if collection_info.points_count > 0 else [],
                with_payload=True,
                with_vectors=False
            )
            
            print(f"Sample records retrieved: {len(records) if records else 0}")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        await db.close_connections()

if __name__ == "__main__":
    asyncio.run(test_qdrant())