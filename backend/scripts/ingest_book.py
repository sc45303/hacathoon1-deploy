#!/usr/bin/env python3
"""
Main ingestion script for the book ingestion pipeline with forced re-ingestion.
"""

import argparse
import asyncio
import logging
import os
import sys
from datetime import datetime
from dotenv import load_dotenv
load_dotenv()

# Add parent directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from app.ingestion.reader import read_markdown_files, extract_text_from_markdown
from app.ingestion.chunker import chunk_text
from app.ingestion.embedder import generate_embedding
from app.ingestion.storage import store_chunk, upsert_embedding
from app.db.qdrant import QdrantDB  # <-- Yeh import add karo

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def main():
    parser = argparse.ArgumentParser(description="Book Ingestion Pipeline")
    parser.add_argument("--source-dir", required=True, help="Path to markdown files directory")
    parser.add_argument("--book-id", required=True, help="Book identifier")
    parser.add_argument("--chunk-size", type=int, default=400)
    parser.add_argument("--overlap", type=int, default=50)
    parser.add_argument("--run-identifier", help="Custom run ID")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force-reingest", action="store_true",  # <-- Nayi flag
                        help="Delete existing collection before ingestion")
    
    args = parser.parse_args()
    
    logger.info(f"Starting ingestion for book: {args.book_id}")
    logger.info(f"Source directory: {args.source_dir}")
    
    run_identifier = args.run_identifier or f"{args.book_id}-{datetime.now().strftime('%Y%m%d-%H%M')}"
    logger.info(f"Run identifier: {run_identifier}")
    
    # Initialize Qdrant client
    qdrant_db = QdrantDB()
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "test-clustor")
    
    # FORCE RE-INGEST: Delete old collection if flag is set
    if args.force_reingest:
        logger.info(f"Force re-ingest enabled: Deleting existing collection '{collection_name}'...")
        try:
            await qdrant_db.delete_collection(collection_name)
            logger.info(f"Successfully deleted collection '{collection_name}'")
        except Exception as e:
            logger.warning(f"Could not delete collection (might not exist): {e}")
    
    # Recreate collection (optional but recommended)
    try:
        await qdrant_db.create_collection(collection_name)
        logger.info(f"Collection '{collection_name}' ready for ingestion")
    except Exception as e:
        logger.info(f"Collection already exists or error creating: {e}")
    
    if args.dry_run:
        logger.info("DRY RUN MODE ENABLED")
    
    total_files = processed_files = total_chunks = chunks_stored = embeddings_stored = 0
    
    try:
        markdown_files = read_markdown_files(args.source_dir)
        total_files = len(markdown_files)
        logger.info(f"Found {total_files} markdown files")
        
        for i, md_file in enumerate(markdown_files):
            logger.info(f"Processing {i+1}/{total_files}: {md_file['file_path']}")
            
            text_content = extract_text_from_markdown(md_file['content'])
            chunks = chunk_text(
                text=text_content,
                source_file=md_file['file_path'],
                chunk_size=args.chunk_size,
                overlap=args.overlap
            )
            
            for j, chunk in enumerate(chunks):
                if args.dry_run:
                    continue
                
                try:
                    # Store in Neon
                    chunk_id = await store_chunk({
                        **chunk,
                        'book_id': args.book_id,
                        'chapter': extract_chapter_from_path(md_file['file_path']),
                        'section': extract_section_from_content(text_content, j)
                    })
                    
                    if chunk_id:
                        chunks_stored += 1
                    
                    # Generate embedding
                    embedding = await generate_embedding(chunk['content'])
                    
                    # PAYLOAD MEIN CONTENT SABSE UPAR AUR ZAROORI
                    payload = {
                        "content": chunk['content'],  # <-- Yeh first rakho
                        "chunk_id": chunk_id,
                        "chunk_hash": chunk['chunk_hash'],
                        "book_id": args.book_id,
                        "chapter": extract_chapter_from_path(md_file['file_path']),
                        "section": extract_section_from_content(text_content, j),
                        "source_file": chunk['source_file'],
                        "chunk_index": chunk['chunk_index']
                    }
                    
                    # Upsert in Qdrant
                    success = await upsert_embedding(embedding, payload)
                    if success:
                        embeddings_stored += 1
                    
                    total_chunks += 1
                    
                except Exception as e:
                    logger.error(f"Error processing chunk: {e}")
                    continue
            
            processed_files += 1
        
        logger.info("=== INGESTION SUMMARY ===")
        logger.info(f"Files processed: {processed_files}/{total_files}")
        logger.info(f"Total chunks: {total_chunks}")
        logger.info(f"Chunks stored: {chunks_stored}")
        logger.info(f"Embeddings stored: {embeddings_stored}")
        logger.info("INGESTION COMPLETE WITH FRESH DATA!")
        
    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        return False
    
    return True


def extract_chapter_from_path(file_path: str) -> str:
    import os.path
    directory = os.path.dirname(file_path)
    return os.path.basename(directory) if directory else "root"


def extract_section_from_content(content: str, chunk_index: int) -> str:
    return f"Section_{chunk_index}"


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)