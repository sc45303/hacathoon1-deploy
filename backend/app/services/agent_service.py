import logging
from typing import List
from app.models.agent import QueryRequest, QueryResponse, Source
from app.rag.pipeline import RAGPipeline
from openai import OpenAI
import os


logger = logging.getLogger(__name__)
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


class AgentService:
    """Service class for the agent that intelligently routes queries based on context."""

    @staticmethod
    def classify_query(request: QueryRequest) -> str:
        """
        Smart mode classification with clear priority:
        1. selected_text → highest priority (if provided and not empty)
        2. general → only for greetings, chit-chat, or obvious non-course questions
        3. book → default for everything related to the course (RAG pipeline)
        """

        question = request.question.strip().lower() if request.question else ""

        # 1. Highest priority: Selected text from UI
        if request.selected_text and request.selected_text.strip():
            logger.info("Mode selected: selected_text (user highlighted text)")
            return "selected_text"

        # 2. General chit-chat / off-topic questions
        general_triggers = {
            "hi", "hello", "hey", "hola", "namaste",
            "who are you", "what is your name", "introduce yourself",
            "how are you", "what's up", "sup",
            "what can you do", "help", "what do you know",
            "tell me a joke", "joke", "funny",
            "thank you", "thanks", "thank u", "thx",
            "good morning", "good afternoon", "good evening", "good night",
            "bye", "goodbye", "see you", "later",
            "what is ai", "who is elon musk", "what is grok", "what is xai",
            "weather", "time", "date", "news"
        }

        # Simple word/phrase match
        if any(trigger in question for trigger in general_triggers):
            logger.info("Mode selected: general (chit-chat/off-topic)")
            return "general"

        # 3. Default: Course/book-related → use RAG
        logger.info("Mode selected: book (default course RAG mode)")
        return "book"

    @staticmethod
    async def handle_selected_text_mode(request: QueryRequest) -> QueryResponse:
        """Answer primarily based on user-selected text, but fallback gracefully if needed."""
        logger.info("Handling query in selected_text mode")

        selected_text = request.selected_text.strip()

        if not selected_text:
            return QueryResponse(
                answer="No text was selected. Highlight some text from the course and ask about it!",
                sources=[],
                mode="selected_text"
            )

        prompt = f"""
You are helping with the Physical AI & Humanoid Robotics course.

Use the following selected text as the primary context to answer the question.
If the answer is clearly in the text, use it.
If not, you can use your general knowledge about the course topics (ROS2, Gazebo, Isaac, etc.), but mention that it's not directly from the selected text.

Selected text:
\"\"\"{selected_text}\"\"\"

Question: {request.question}

Answer clearly and helpfully.
"""

        try:
            response = client.chat.completions.create(
                model="gpt-4o-mini",  # Ya gpt-3.5-turbo rakh
                messages=[
                    {"role": "system", "content": "You are a helpful course assistant. Be accurate and friendly."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=600,
                temperature=0.4  # Thoda low for accuracy
            )

            answer = response.choices[0].message.content.strip()

            source_preview = selected_text[:300] + ("..." if len(selected_text) > 300 else "")

            sources = [
                Source(
                    document_id="user_selected_text",
                    content=source_preview,
                    page_number=None,
                    section_title="Highlighted Text from Course"
                )
            ]

            return QueryResponse(
                answer=answer,
                sources=sources,
                mode="selected_text"
            )

        except Exception as e:
            logger.error(f"Error in selected_text mode: {e}")
            return QueryResponse(
                answer="Sorry, I couldn't process the selected text right now. Try asking directly!",
                sources=[],
                mode="selected_text"
            )
    @staticmethod
    async def handle_book_rag_mode(request: QueryRequest) -> QueryResponse:
        """Use full RAG pipeline on the course book."""
        logger.info("Handling query in book RAG mode")

        try:
            pipeline = RAGPipeline()
            result = await pipeline.query(request.question)

            formatted_sources = []
            for source in result.get('sources', []):
                formatted_sources.append(Source(
                    document_id=source.get("source_file", "unknown"),
                    content=source.get("content", "")[:200] + "..." if source.get("content") else "",
                    page_number=source.get("page_number"),
                    section_title=source.get("section", "")
                ))

            return QueryResponse(
                answer=result['answer'],
                sources=formatted_sources,
                mode="book"
            )

        except Exception as e:
            logger.error(f"Error in book RAG mode: {e}")
            return QueryResponse(
                answer="I'm having trouble accessing the course materials right now. Please try again.",
                sources=[],
                mode="book"
            )

    @staticmethod
    async def handle_general_mode(request: QueryRequest) -> QueryResponse:
        """Pure LLM answer – no book, no RAG."""
        logger.info("Handling query in general mode")

        try:
            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": "You are a friendly and helpful AI assistant."},
                    {"role": "user", "content": request.question}
                ],
                max_tokens=500,
                temperature=0.7
            )

            answer = response.choices[0].message.content.strip()

            return QueryResponse(
                answer=answer,
                sources=[],
                mode="general"
            )

        except Exception as e:
            logger.error(f"Error in general mode: {e}")
            return QueryResponse(
                answer="Oops! I couldn't think of a response right now.",
                sources=[],
                mode="general"
            )

    @staticmethod
    async def process_query(request: QueryRequest) -> QueryResponse:
        """Main entry point – classifies and routes the query."""
        mode = AgentService.classify_query(request)

        if mode == "selected_text":
            return await AgentService.handle_selected_text_mode(request)
        elif mode == "general":
            return await AgentService.handle_general_mode(request)
        else:  # book mode (default)
            return await AgentService.handle_book_rag_mode(request)