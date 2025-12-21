"""
Answer generation module for the RAG (Retrieval Augmented Generation) system.

This module provides async methods to generate answers using OpenAI's
Chat Completion API based on the provided context and user question.
"""
from typing import Dict, Any, Optional
import openai
import logging
from openai import AsyncOpenAI
from dotenv import load_dotenv
import os
from app.core.config import settings


logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Initialize the OpenAI client
client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Use the model specified in the settings, defaulting to gpt-3.5-turbo
OPENAI_MODEL = getattr(settings, "OPENAI_MODEL", "gpt-3.5-turbo")


class AnswerGenerator:
    """
    Generates answers using OpenAI's Chat Completion API based on context.
    """
    
    def __init__(self, temperature: float = 0.3):
        """
        Initialize the AnswerGenerator with a specific temperature.
        
        Args:
            temperature: Controls randomness in generation (0.0-1.0, lower means less random)
        """
        if temperature > 0.3:
            logger.warning(f"Temperature {temperature} is higher than recommended maximum of 0.3 for RAG application")
        self.temperature = temperature

    async def generate_answer(
        self, 
        system_message: str, 
        user_message: str,
        max_tokens: int = 1000
    ) -> Optional[Dict[str, Any]]:
        """
        Generate an answer using OpenAI Chat Completion API.
        
        Args:
            system_message: The system message providing context and instructions
            user_message: The user message containing the question and context
            max_tokens: Maximum number of tokens to generate (default: 1000)
            
        Returns:
            Dictionary containing the response or None if generation failed
        """
        try:
            response = await client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                temperature=self.temperature,
                max_tokens=max_tokens,
                timeout=30  # 30 second timeout
            )
            
            # Extract the answer from the response
            answer = response.choices[0].message.content
            usage = {
                "prompt_tokens": response.usage.prompt_tokens if response.usage else 0,
                "completion_tokens": response.usage.completion_tokens if response.usage else 0,
                "total_tokens": response.usage.total_tokens if response.usage else 0
            }
            
            result = {
                "answer": answer,
                "usage": usage,
                "model": response.model
            }
            
            logger.info(f"Successfully generated answer with {usage['total_tokens']} total tokens used")
            return result
            
        except openai.APIError as e:
            logger.error(f"OpenAI API error during answer generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during answer generation: {e}")
            return None

    async def generate_answer_simple(self, prompt: str) -> Optional[str]:
        """
        Generate an answer using a simple prompt format.
        
        Args:
            prompt: Complete prompt string including system instructions and user question
            
        Returns:
            Generated answer text or None if generation failed
        """
        try:
            response = await client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                timeout=30  # 30 second timeout
            )
            
            answer = response.choices[0].message.content
            logger.info(f"Successfully generated answer with model {response.model}")
            return answer
            
        except openai.APIError as e:
            logger.error(f"OpenAI API error during simple answer generation: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during simple answer generation: {e}")
            return None