"""
Crawler Module

This module handles the crawling of websites to extract all accessible URLs.
"""
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import logging
from typing import List


def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl the website and return a list of all accessible URLs
    
    Args:
        base_url: The base URL of the website to crawl
        
    Returns:
        List of all accessible URLs found on the website
    """
    visited_urls = set()
    urls_to_visit = [base_url]
    
    while urls_to_visit:
        current_url = urls_to_visit.pop(0)
        
        # Skip if already visited
        if current_url in visited_urls:
            continue

        # Only process URLs from the same domain
        if not current_url.startswith(base_url):
            continue
            
        try:
            response = requests.get(current_url, timeout=10)
            response.raise_for_status()
            
            # Mark as visited
            visited_urls.add(current_url)
            
            # Parse HTML to find links
            soup = BeautifulSoup(response.text, 'html.parser')
            
            # Find all links on the page
            for link in soup.find_all('a', href=True):
                href = link['href']
                full_url = urljoin(current_url, href)
                
                # Make sure the URL is within the same domain
                if full_url.startswith(base_url) and full_url not in visited_urls:
                    urls_to_visit.append(full_url)
        except Exception as e:
            logging.warning(f"Could not crawl {current_url}: {str(e)}")
            continue

    return list(visited_urls)