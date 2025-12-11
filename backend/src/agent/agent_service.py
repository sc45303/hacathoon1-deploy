"""
OpenAI agent service for the retrieval-enabled agent.

This module implements the core OpenAI agent that uses a custom retrieval tool
to answer questions about the book content.
"""
import logging
import openai
from typing import List, Dict, Any, Optional
from backend.src.agent.config import AgentConfig
from backend.src.agent.retrieval_tool import RetrievalTool
from backend.src.agent.models import RetrievedChunk, AgentResponse
from backend.src.agent.utils import generate_id, log_agent_interaction, Timer
from datetime import datetime


class AgentService:
    """
    Service class for managing the OpenAI agent with retrieval capabilities.
    
    This class orchestrates the interaction between the OpenAI agent and the
    custom retrieval tool to answer questions based on book content.
    """
    
    def __init__(self, config: AgentConfig):
        """
        Initialize the agent service with configuration and tools.
        
        Args:
            config: Agent configuration containing API keys and settings
        """
        self.config = config
        
        # Set OpenAI API key
        openai.api_key = config.openai_api_key
        
        # Initialize retrieval tool
        self.retrieval_tool = RetrievalTool(config)
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
        # Create OpenAI assistant with the retrieval tool
        self.assistant = openai.beta.assistants.create(
            name="Book Retrieval Assistant",
            instructions=(
                "You are a helpful assistant that answers questions based solely on the provided book content. "
                "Use the retrieval_tool to find relevant information from the book. "
                "Always cite the specific sources you used to answer the question. "
                "If you cannot find relevant information in the provided content, state that clearly."
            ),
            model=config.agent_model,
            tools=[
                {
                    "type": "function",
                    "function": {
                        "name": "retrieval_tool",
                        "description": "Retrieve relevant text chunks from the book based on a query",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "query": {
                                    "type": "string",
                                    "description": "The query to search for relevant text chunks"
                                }
                            },
                            "required": ["query"]
                        }
                    }
                }
            ]
        )
        
    def create_thread(self) -> str:
        """
        Create a new thread for the conversation.
        
        Returns:
            Thread ID for the newly created thread
        """
        thread = openai.beta.threads.create()
        return thread.id
    
    def ask_question(self, query: str, thread_id: Optional[str] = None) -> AgentResponse:
        """
        Process a question by interacting with the OpenAI agent and retrieval tool.
        
        Args:
            query: The user's question
            thread_id: Optional thread ID to maintain conversation context
            
        Returns:
            AgentResponse with the answer and retrieved sources
        """
        log_agent_interaction("agent_call_start", {"query": query}, self.logger)
        
        with Timer() as timer:
            try:
                # Create a thread if not provided
                if thread_id is None:
                    thread_id = self.create_thread()
                
                # Add the user's message to the thread
                openai.beta.threads.messages.create(
                    thread_id=thread_id,
                    role="user",
                    content=query
                )
                
                # Run the assistant
                run = openai.beta.threads.runs.create(
                    thread_id=thread_id,
                    assistant_id=self.assistant.id,
                    # Adding timeout if supported by the API
                    # Note: The timeout_seconds from config is for API calls, not for run execution
                )
                
                # Wait for the run to complete (this is a simplified approach)
                # In a production system, we'd want to poll for completion or use streaming
                import time
                while run.status in ["queued", "in_progress"]:
                    time.sleep(0.5)  # Poll every 0.5 seconds
                    run = openai.beta.threads.runs.retrieve(
                        thread_id=thread_id,
                        run_id=run.id
                    )
                
                # Check if the run completed successfully
                if run.status == "completed":
                    # Retrieve the messages
                    messages = openai.beta.threads.messages.list(
                        thread_id=thread_id,
                        order="desc",
                        limit=1  # Get the latest message
                    )
                    
                    if messages.data:
                        agent_response_content = messages.data[0].content[0].text.value
                        
                        # Extract cited sources from the run steps (if available)
                        run_steps = openai.beta.threads.runs.steps.list(
                            thread_id=thread_id,
                            run_id=run.id
                        )
                        
                        # For now, manually retrieve chunks used based on tool calls during the run
                        # In a more sophisticated implementation, we'd track which chunks were retrieved
                        # based on the tool call details
                        retrieved_chunks = self._get_retrieved_chunks_from_query(query)
                        
                        response_id = generate_id("response")
                        query_id = generate_id("query")
                        
                        agent_response = AgentResponse(
                            response_id=response_id,
                            content=agent_response_content,
                            query_id=query_id,
                            chunks_used=retrieved_chunks,
                            timestamp=datetime.now(),
                            confidence_score=None  # Confidence would need to be computed based on relevance scores
                        )
                        
                        log_agent_interaction("agent_call_end", {
                            "query": query,
                            "response": agent_response_content,
                            "chunks_used_count": len(retrieved_chunks),
                            "execution_time": timer.elapsed
                        }, self.logger)
                        
                        return agent_response
                    else:
                        raise Exception("No response generated by the agent")
                else:
                    error_msg = f"Run failed with status: {run.status}"
                    if hasattr(run, 'last_error') and run.last_error:
                        error_msg += f", Error: {run.last_error}"
                    
                    log_agent_interaction("agent_call_error", {
                        "query": query,
                        "error": error_msg
                    }, self.logger)
                    
                    raise Exception(error_msg)
                    
            except Exception as e:
                self.logger.error(f"Error in agent service: {e}")
                log_agent_interaction("agent_call_error", {
                    "query": query,
                    "error": str(e)
                }, self.logger)
                raise
    
    def _get_retrieved_chunks_from_query(self, query: str) -> List[RetrievedChunk]:
        """
        Helper method to retrieve chunks based on the original query.
        
        Args:
            query: The original query to retrieve chunks for
            
        Returns:
            List of RetrievedChunk objects
        """
        return self.retrieval_tool.retrieve_chunks(query)
    
    def run_agent_with_instructions(self, instructions: str, input_data: Optional[Dict[str, Any]] = None) -> AgentResponse:
        """
        Run the agent with custom instructions instead of a simple query.
        
        Args:
            instructions: Custom instructions for the agent
            input_data: Additional data to provide to the agent
            
        Returns:
            AgentResponse with the agent's output
        """
        log_agent_interaction("agent_run_start", {"instructions": instructions}, self.logger)
        
        with Timer() as timer:
            try:
                # Create a thread for this run
                thread_id = self.create_thread()
                
                # Add the instructions and input data to the thread
                content = f"{instructions}\n\nAdditional input data: {input_data}" if input_data else instructions
                openai.beta.threads.messages.create(
                    thread_id=thread_id,
                    role="user",
                    content=content
                )
                
                # Run the assistant
                run = openai.beta.threads.runs.create(
                    thread_id=thread_id,
                    assistant_id=self.assistant.id
                )
                
                # Wait for the run to complete
                import time
                while run.status in ["queued", "in_progress"]:
                    time.sleep(0.5)  # Poll every 0.5 seconds
                    run = openai.beta.threads.runs.retrieve(
                        thread_id=thread_id,
                        run_id=run.id
                    )
                
                # Check if the run completed successfully
                if run.status == "completed":
                    # Retrieve the messages
                    messages = openai.beta.threads.messages.list(
                        thread_id=thread_id,
                        order="desc",
                        limit=1  # Get the latest message
                    )
                    
                    if messages.data:
                        agent_output = messages.data[0].content[0].text.value
                        
                        # For this run, get any chunks that might have been used
                        # This is a simplified implementation - in practice, you'd want to track
                        # which tools were called during the run
                        retrieved_chunks = []
                        
                        response_id = generate_id("response")
                        query_id = generate_id("query")
                        
                        agent_response = AgentResponse(
                            response_id=response_id,
                            content=agent_output,
                            query_id=query_id,
                            chunks_used=retrieved_chunks,
                            timestamp=datetime.now(),
                            confidence_score=None
                        )
                        
                        log_agent_interaction("agent_run_end", {
                            "instructions": instructions,
                            "output": agent_output,
                            "execution_time": timer.elapsed
                        }, self.logger)
                        
                        return agent_response
                    else:
                        raise Exception("No response generated by the agent")
                else:
                    error_msg = f"Run failed with status: {run.status}"
                    if hasattr(run, 'last_error') and run.last_error:
                        error_msg += f", Error: {run.last_error}"
                    
                    log_agent_interaction("agent_run_error", {
                        "instructions": instructions,
                        "error": error_msg
                    }, self.logger)
                    
                    raise Exception(error_msg)
                    
            except Exception as e:
                self.logger.error(f"Error in agent run: {e}")
                log_agent_interaction("agent_run_error", {
                    "instructions": instructions,
                    "error": str(e)
                }, self.logger)
                raise
    
    def check_health(self) -> Dict[str, Any]:
        """
        Check the health of the agent service and its dependencies.
        
        Returns:
            Dictionary with health status information
        """
        health_status = {
            "openai": {"status": "unknown", "response_time_ms": 0},
            "qdrant": {"status": "unknown", "response_time_ms": 0},
            "cohere": {"status": "unknown", "response_time_ms": 0}
        }
        
        # Check OpenAI API
        try:
            import time
            start_time = time.time()
            # Make a simple API call to check if the API is accessible
            openai.models.list(limit=1)
            end_time = time.time()
            health_status["openai"]["status"] = "available"
            health_status["openai"]["response_time_ms"] = (end_time - start_time) * 1000
        except Exception as e:
            self.logger.error(f"OpenAI health check failed: {e}")
            health_status["openai"]["status"] = "unavailable"
            health_status["openai"]["response_time_ms"] = 0
        
        # Check Qdrant (delegating to retrieval tool)
        try:
            start_time = time.time()
            # Test retrieval with a simple query
            self.retrieval_tool.retrieve_chunks("test")
            end_time = time.time()
            health_status["qdrant"]["status"] = "available"
            health_status["qdrant"]["response_time_ms"] = (end_time - start_time) * 1000
        except Exception as e:
            self.logger.error(f"Qdrant health check failed: {e}")
            health_status["qdrant"]["status"] = "unavailable"
            health_status["qdrant"]["response_time_ms"] = 0
        
        # Check Cohere (delegating to retrieval tool)
        try:
            start_time = time.time()
            # Test Cohere embedding generation through the retrieval tool
            self.retrieval_tool._generate_embedding("test")
            end_time = time.time()
            health_status["cohere"]["status"] = "available"
            health_status["cohere"]["response_time_ms"] = (end_time - start_time) * 1000
        except Exception as e:
            self.logger.error(f"Cohere health check failed: {e}")
            health_status["cohere"]["status"] = "unavailable"
            health_status["cohere"]["response_time_ms"] = 0
        
        # Determine overall status
        overall_status = "healthy"
        for service in health_status.values():
            if service["status"] == "unavailable":
                overall_status = "degraded"
                break
        
        return {
            "status": overall_status,
            "services": health_status
        }