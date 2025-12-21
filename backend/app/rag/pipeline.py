"""
RAG (Retrieval Augmented Generation) pipeline orchestrator.

This module combines all RAG components into a single workflow:
1. Query embedding
2. Vector retrieval
3. Direct context from Qdrant payload (no Neon DB needed)
4. Prompt construction
5. Answer generation
"""
from typing import Dict, Any, List, Optional
import logging

from app.rag.retriever import VectorRetriever
from app.rag.prompt import RAGPromptEngine
from app.rag.generator import AnswerGenerator
from app.utils.embeddings import get_embedding


logger = logging.getLogger(__name__)


class RAGPipeline:
    """
    Orchestrates the complete RAG flow from question to answer.
    Now uses content directly from Qdrant payload – no Neon DB fetch required.
    """
    
    def __init__(self):
        self.retriever = VectorRetriever()
        self.prompt_engine = RAGPromptEngine()
        self.generator = AnswerGenerator(temperature=0.3)
        # self.db = NeonDB()  # ← No longer needed for RAG queries

    async def query(self, question: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Execute the full RAG pipeline to answer a question.
        """
        try:
            # Step 1: Retrieve relevant chunks from Qdrant
            retrieved_chunks = await self.retriever.retrieve_similar_chunks(
                query_text=question,
                top_k=top_k
            )
            
            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for question: {question[:50]}...")

            # Step 2: Extract context directly from Qdrant payloads
            context_for_llm = []
            for chunk in retrieved_chunks:
                payload = chunk.get('payload', {})
                content = payload.get('content')

                if content and content.strip():
                    context_for_llm.append({
                        'content': content.strip(),
                        'book_id': payload.get('book_id', ''),
                        'chapter': payload.get('chapter', ''),
                        'section': payload.get('section', ''),
                        'chunk_id': payload.get('chunk_id')
                    })
                else:
                    chunk_id = chunk.get('id', 'unknown')
                    logger.warning(f"Chunk {chunk_id} is missing 'content' in payload")

            if context_for_llm:
                logger.info(f"Successfully loaded {len(context_for_llm)} chunks directly from Qdrant payloads")
            else:
                logger.info("No chunks with valid content found in Qdrant results")

            # Step 3: Construct the RAG prompt
            system_message = self.prompt_engine.construct_system_message()
            user_message = self.prompt_engine.construct_user_message(
                context_chunks=context_for_llm,
                user_question=question
            )
            
            # Step 4: Generate the answer using the LLM
            result = await self.generator.generate_answer(
                system_message=system_message,
                user_message=user_message
            )
            
            if result is None:
                logger.error("Answer generation failed, returning fallback message")
                answer = "Answer not found in provided content"
            else:
                answer = result['answer']
            
            # Step 5: Format sources from Qdrant payloads
            sources = []
            for chunk in retrieved_chunks:
                payload = chunk.get('payload', {})
                source = {
                    'source_file': payload.get('source_file', ''),
                    'chapter': payload.get('chapter', ''),
                    'section': payload.get('section', '')
                }
                if any(source.values()):  # Only add if at least one field has value
                    sources.append(source)
            
            # Remove duplicates while preserving order
            seen = set()
            unique_sources = []
            for source in sources:
                key = (source['source_file'], source['chapter'], source['section'])
                if key not in seen:
                    seen.add(key)
                    unique_sources.append(source)
            
            logger.info(f"RAG pipeline completed successfully, with {len(unique_sources)} unique sources")
            
            return {
                'answer': answer,
                'sources': unique_sources
            }
            
        except Exception as e:
            logger.error(f"Error in RAG pipeline: {e}")
            return {
                'answer': "An error occurred while processing your question. Please try again.",
                'sources': []
            }