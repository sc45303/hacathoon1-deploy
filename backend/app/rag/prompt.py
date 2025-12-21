"""
RAG prompt engineering module for the AI book application.

This module provides functions to construct prompts for the RAG system
that enforce context-only answers and prevent hallucinations.
"""
from typing import List, Dict, Any
import logging


logger = logging.getLogger(__name__)


class RAGPromptEngine:
    """
    Engine for constructing RAG prompts with strict rules to prevent hallucinations.
    """
    
    @staticmethod
    def construct_rag_prompt(context_chunks: List[Dict[str, Any]], user_question: str) -> str:
        """
        Construct a RAG prompt with the retrieved context and user question.
        
        Args:
            context_chunks: List of context chunks retrieved from the database
            user_question: The original question from the user
            
        Returns:
            Formatted prompt string for the LLM
        """
        # Build the context string from the retrieved chunks
        context_parts = []
        for i, chunk in enumerate(context_chunks):
            content = chunk.get('content', '').strip()
            if content:  # Only add non-empty content
                context_parts.append(f"Context {i+1}: {content}")
        
        context_text = "\n\n".join(context_parts)
        
        # If no context is available, return a prompt that indicates no answer can be provided
        if not context_text:
            return RAGPromptEngine.get_no_context_prompt(user_question)
        
        # Construct the full prompt with strict instructions
        prompt = f"""
You are a helpful assistant that answers questions based ONLY on the provided context from book content.
DO NOT use any external knowledge or information not present in the context below.
If the answer is not found in the provided context, respond with "Answer not found in provided content".

CONTEXT:
{context_text}

QUESTION:
{user_question}

ANSWER:
"""
        return prompt.strip()
    
    @staticmethod
    def get_no_context_prompt(user_question: str) -> str:
        """
        Return a prompt for when there is no relevant context found.
        
        Args:
            user_question: The original question from the user
            
        Returns:
            Prompt indicating no context is available
        """
        return f"""
You are a helpful assistant. 
The user asked: "{user_question}"
However, no relevant information was found in the provided book content to answer this question.
Respond with: "Answer not found in provided content".
"""
    
    @staticmethod
    def construct_system_message() -> str:
        """
        Construct the system message that enforces the RAG rules.
        
        Returns:
            System message for the LLM
        """
        return """
You are a helpful assistant that answers questions based ONLY on the specific context provided to you.
You must follow these rules:
1. Answer ONLY using information from the provided context
2. DO NOT use any external knowledge, general knowledge, or knowledge not in the provided context
3. If the provided context does not contain information needed to answer the question, respond with "Answer not found in provided content"
4. Do not make up, infer, or hallucinate any information not explicitly in the provided context
5. Answer in a clear, concise, and helpful manner
6. Keep responses professional and focused on the question
"""
    
    @staticmethod
    def construct_user_message(context_chunks: List[Dict[str, Any]], user_question: str) -> str:
        """
        Construct the user message combining context and question.
        
        Args:
            context_chunks: List of retrieved context chunks
            user_question: The original question from the user
            
        Returns:
            User message for the LLM
        """
        # Build the context string from the retrieved chunks
        context_parts = []
        for i, chunk in enumerate(context_chunks):
            content = chunk.get('content', '').strip()
            book_id = chunk.get('book_id', 'Unknown')
            chapter = chunk.get('chapter', 'Unknown')
            section = chunk.get('section', 'Unknown')
            
            if content:  # Only add non-empty content
                context_parts.append(f"Source: Book {book_id}, Chapter {chapter}, Section {section}\nContent: {content}")
        
        context_text = "\n\n".join(context_parts)
        
        if not context_text:
            # No context available
            return f"""
Based on the following question, but with no relevant context found in the provided content:

Question: {user_question}

Response: Answer not found in provided content
"""
        else:
            # Context is available
            return f"""
Using only the following context, please answer the question:

CONTEXT:
{context_text}

QUESTION:
{user_question}

ANSWER:
"""