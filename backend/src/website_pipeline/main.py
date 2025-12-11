"""
Website Embedding Pipeline

This module implements a complete pipeline that:
1. Crawls a Docusaurus-deployed book website
2. Extracts clean text from all pages
3. Chunks the content appropriately
4. Generates embeddings using Cohere models
5. Stores embeddings in a Qdrant collection named 'reg-embedding'
"""
import os
import logging
import time
from dataclasses import dataclass
from typing import List, Dict, Any, Optional

from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import from local modules
from website_pipeline.crawler import get_all_urls
from website_pipeline.text_extractor import extract_text_from_url
from website_pipeline.chunker import chunk_text
from website_pipeline.embedder import embed
from website_pipeline.storage import create_collection, save_chunk_to_qdrant


@dataclass
class TextSegment:
    """Represents a chunk of text with metadata"""
    id: str
    content: str
    url: str
    position: int
    metadata: Dict[str, Any]


def main():
    """
    Execute the complete pipeline: crawl -> extract -> chunk -> embed -> store
    """
    # The target website to crawl
    website_url = "https://hacathoon1-deploy.vercel.app/"
    
    # Collection name
    collection_name = "reg-embedding"
    
    print(f"Starting website embedding pipeline for {website_url}")
    
    # Step 1: Create the collection in Qdrant
    print("Creating Qdrant collection...")
    if not create_collection(collection_name):
        print("Failed to create Qdrant collection. Exiting.")
        return
    
    # Step 2: Crawl the website to get all URLs
    print("Crawling website to get all URLs...")
    urls = get_all_urls(website_url)
    print(f"Found {len(urls)} URLs to process")
    
    # Step 3: Process each URL
    total_segments = 0
    for i, url in enumerate(urls):
        print(f"Processing URL {i+1}/{len(urls)}: {url}")
        
        # Extract text from URL
        text = extract_text_from_url(url)
        if not text.strip():
            print(f"No text extracted from {url}, skipping...")
            continue
        
        # Chunk the text
        text_chunks = chunk_text(text)
        
        # Embed each chunk
        embeddings = embed(text_chunks)
        
        # Save each chunk to Qdrant
        for j, (chunk, embedding) in enumerate(zip(text_chunks, embeddings)):
            metadata = {
                "url": url,
                "position": j,
                "source": "website_embedding_pipeline"
            }
            
            success = save_chunk_to_qdrant(chunk, embedding, metadata, collection_name)
            if success:
                total_segments += 1
            else:
                print(f"Failed to save chunk {j} from {url}")
    
    print(f"Pipeline completed successfully! Stored {total_segments} text segments in Qdrant collection '{collection_name}'")


if __name__ == "__main__":
    main()