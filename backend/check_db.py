import asyncio
import asyncpg
from app.core.config import settings
import sys

async def test_connection():
    print(f"Connecting to: {settings.NEON_DATABASE_URL[:50]}...")
    try:
        conn = await asyncpg.connect(settings.NEON_DATABASE_URL)
        print("Connected successfully!")
        
        # Check if table exists
        table_check = await conn.fetch('''
            SELECT table_name 
            FROM information_schema.tables 
            WHERE table_name = 'book_content_chunks'
        ''')
        print(f"Table exists: {len(table_check) > 0}")
        
        if len(table_check) > 0:
            # Count total records
            result = await conn.fetch('SELECT COUNT(*) FROM book_content_chunks')
            print(f"Total chunks in database: {result[0]['count']}")
            
            # Check for our book
            result = await conn.fetch('SELECT COUNT(*) FROM book_content_chunks WHERE book_id = $1', 'hacathon-book')
            print(f"Total chunks for hacathon-book: {result[0]['count']}")
        
        await conn.close()
        print("Connection closed successfully")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_connection())