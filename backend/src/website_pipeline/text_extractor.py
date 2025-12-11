"""
Text Extractor Module

This module handles the extraction of clean text from web pages.
"""
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin
import logging
from typing import List


def extract_text_from_url(url: str) -> str:
    """
    Extract clean text content from a given URL
    
    Args:
        url: The URL to extract text from
        
    Returns:
        Clean text content from the URL
    """
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.text, 'html.parser')
        
        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()
        
        # Get text content, focusing on main content areas
        # Prioritize content in main, article, or content-specific divs
        main_content = (
            soup.find('main') or 
            soup.find('article') or 
            soup.find('div', class_='main-content') or
            soup.find('div', class_='content') or
            soup.find('div', {'id': 'content'}) or
            soup.find('div', class_=lambda x: x and 'doc' in x.lower()) or
            soup.find('div', class_=lambda x: x and 'container' in x.lower()) or
            soup
        )
        
        # Extract text
        text = main_content.get_text()
        
        # Clean up text by removing extra whitespace
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)
        
        return text
    except Exception as e:
        logging.error(f"Could not extract text from {url}: {str(e)}")
        return ""