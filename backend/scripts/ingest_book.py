#!/usr/bin/env python3
"""
Main ingestion script for the book ingestion pipeline.

This script orchestrates the full ingestion flow:
1. Reads markdown files from a directory
2. Parses and extracts text content
3. Chunks the text
4. Generates embeddings
5. Stores content and embeddings in respective databases
"""

import argparse
import asyncio
import logging
import os
import sys
from datetime import datetime
from typing import Dict, List, Any
from dotenv import load_dotenv
load_dotenv()
# Add parent directory to path to import ingestion modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from app.ingestion.reader import read_markdown_files, extract_text_from_markdown
from app.ingestion.chunker import chunk_text, calculate_tokens
from app.ingestion.embedder import generate_embedding
from app.ingestion.storage import store_chunk, upsert_embedding, chunk_exists

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def main():
    parser = argparse.ArgumentParser(description="Book Ingestion Pipeline")
    parser.add_argument("--source-dir", required=True, 
                        help="Path to the directory containing markdown files")
    parser.add_argument("--book-id", required=True, 
                        help="Identifier for the book being ingested")
    parser.add_argument("--chunk-size", type=int, default=400,
                        help="Number of tokens per chunk (default: 400)")
    parser.add_argument("--overlap", type=int, default=50,
                        help="Number of overlapping tokens (default: 50)")
    parser.add_argument("--run-identifier", 
                        help="Custom identifier for this ingestion run")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run script without writing to databases")
    
    args = parser.parse_args()
    
    logger.info(f"Starting ingestion for book: {args.book_id}")
    logger.info(f"Source directory: {args.source_dir}")
    logger.info(f"Chunk size: {args.chunk_size}, Overlap: {args.overlap}")
    
    if args.dry_run:
        logger.info("DRY RUN MODE: No database writes will occur")
    
    # Generate a run identifier if not provided
    run_identifier = args.run_identifier or f"{args.book_id}-{datetime.now().strftime('%Y-%m-%d-%H-%M')}"
    logger.info(f"Run identifier: {run_identifier}")
    
    # Initialize counters for progress tracking
    total_files = 0
    processed_files = 0
    total_chunks = 0
    chunks_stored = 0
    embeddings_stored = 0
    
    try:
        # Step 1: Read markdown files
        logger.info("Step 1: Reading markdown files...")
        markdown_files = read_markdown_files(args.source_dir)
        total_files = len(markdown_files)
        logger.info(f"Found {total_files} markdown files")
        
        # Process each markdown file
        for i, md_file in enumerate(markdown_files):
            logger.info(f"Processing file {i+1}/{total_files}: {md_file['file_path']}")
            
            # Extract text content from markdown
            text_content = extract_text_from_markdown(md_file['content'])
            
            # Chunk the text
            chunks = chunk_text(
                text=text_content,
                source_file=md_file['file_path'],
                chunk_size=args.chunk_size,
                overlap=args.overlap
            )
            
            # Process each chunk
            for j, chunk in enumerate(chunks):
                logger.debug(f"Processing chunk {j+1}/{len(chunks)} of {md_file['file_path']}")
                
                # Check for idempotency - skip if chunk already exists
                chunk_exists = False
                    
                # If in dry run mode, skip database operations
                if args.dry_run:
                    logger.info(f"Dry run: Would process chunk {chunk['chunk_id']}")
                    continue
                
                try:
                    # Store text chunk in Neon Postgres
                    chunk_id = await store_chunk({
                        **chunk,
                        'book_id': args.book_id,
                        'chapter': extract_chapter_from_path(md_file['file_path']),  # Extract chapter from path
                        'section': extract_section_from_content(text_content, j)   # Extract section from content
                    })
                    
                    if chunk_id:
                        chunks_stored += 1
                        logger.debug(f"Stored chunk {chunk_id}")
                    
                    # Generate embedding for the chunk
                    embedding = await generate_embedding(chunk['content'])
                    
                    # Prepare payload for Qdrant with reference to Neon record
                    payload = {
                    "content": chunk['content'],      # ← Yeh nayi line sabse upar
                    "chunk_id": chunk_id,
                    "chunk_hash": chunk['chunk_hash'],
                    "book_id": args.book_id,
                    "chapter": extract_chapter_from_path(md_file['file_path']),
                    "section": extract_section_from_content(text_content, j),
                    "source_file": chunk['source_file'],
                    "chunk_index": chunk['chunk_index']
}
                    
                    # Store embedding in Qdrant
                    success = await upsert_embedding(embedding, payload)
                    
                    if success:
                        embeddings_stored += 1
                        logger.debug(f"Stored embedding for chunk {chunk_id}")
                    
                    total_chunks += 1
                    
                except Exception as e:
                    logger.error(f"Error processing chunk from {md_file['file_path']}: {e}")
                    # Continue with other chunks/files instead of stopping
                    continue
            
            processed_files += 1
            logger.info(f"Completed processing {md_file['file_path']}")
        
        # Print summary
        logger.info("=== INGESTION SUMMARY ===")
        logger.info(f"Files processed: {processed_files}/{total_files}")
        logger.info(f"Chunks processed: {total_chunks}")
        logger.info(f"Chunks stored in Neon: {chunks_stored}")
        logger.info(f"Embeddings stored in Qdrant: {embeddings_stored}")
        
        if args.dry_run:
            logger.info("DRY RUN COMPLETE: No data was written to databases")
        else:
            logger.info("INGESTION COMPLETE: Data successfully written to databases")
        
        return True
    
    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        return False


def extract_chapter_from_path(file_path: str) -> str:
    """
    Extract chapter information from the file path.
    This is a simple implementation - in practice you might have more sophisticated logic.
    """
    # Example: if path is 'docs/python-basics/variables.md', extract 'python-basics'
    import os.path
    directory = os.path.dirname(file_path)
    if directory:
        return os.path.basename(directory)
    else:
        return "root"


def extract_section_from_content(content: str, chunk_index: int) -> str:
    """
    Extract section information from content.
    This is a simple implementation - in practice you might parse headers or other structure.
    """
    # For now, just return a default section or derive from content
    return f"Section_{chunk_index}"


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)