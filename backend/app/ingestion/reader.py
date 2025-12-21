"""
Markdown file reader module for the book ingestion pipeline.

This module provides functions to read markdown files from a directory
structure and extract plain text content while preserving metadata.
"""

import os
import logging
from pathlib import Path
from typing import Dict, List, Any
import markdown

logger = logging.getLogger(__name__)


def read_markdown_files(source_dir: str) -> List[Dict[str, Any]]:
    """
    Reads all markdown files from a specified directory and its subdirectories.

    Args:
        source_dir: String path to the directory containing markdown files

    Returns:
        List of dictionaries containing:
        - file_path: Relative path of the markdown file from the source directory
        - content: Raw content of the markdown file
        - metadata: Dictionary with file metadata (size, modification date, etc.)

    Error handling: Raises exception if directory doesn't exist or is inaccessible
    """
    if not os.path.isdir(source_dir):
        raise FileNotFoundError(f"Source directory does not exist: {source_dir}")

    markdown_files = []
    source_path = Path(source_dir)

    for md_file_path in source_path.rglob("*.md"):
        try:
            # Calculate relative path from source directory
            relative_path = md_file_path.relative_to(source_path)

            # Read the file content
            with open(md_file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # Gather file metadata
            stat_info = md_file_path.stat()
            metadata = {
                "size": stat_info.st_size,
                "modified": stat_info.st_mtime,
                "created": stat_info.st_ctime
            }

            markdown_files.append({
                "file_path": str(relative_path),
                "content": content,
                "metadata": metadata
            })

            logger.info(f"Processed markdown file: {relative_path}")

        except Exception as e:
            logger.error(f"Error processing file {md_file_path}: {e}")
            raise

    logger.info(f"Total markdown files processed: {len(markdown_files)}")
    return markdown_files


def extract_text_from_markdown(markdown_content: str) -> str:
    """
    Strips markdown syntax and extracts plain text content.

    Args:
        markdown_content: Raw markdown content as string

    Returns:
        Plain text content with markdown syntax removed

    Error handling: Returns original content if parsing fails
    """
    try:
        # Convert markdown to HTML first
        html_content = markdown.markdown(markdown_content)

        # Remove HTML tags to get plain text
        # This is a simple approach - in a more sophisticated implementation
        # you might want to use BeautifulSoup or similar
        import re
        clean_text = re.sub('<[^<]+?>', '', html_content)
        
        # Clean up extra whitespace
        clean_text = re.sub(r'\n\s*\n', '\n\n', clean_text)
        clean_text = clean_text.strip()
        
        return clean_text
    except Exception as e:
        logger.warning(f"Error extracting text from markdown: {e}. Returning original content.")
        return markdown_content