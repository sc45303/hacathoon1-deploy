from pydantic import BaseModel, Field
from typing import List, Optional


class Source(BaseModel):
    """A document reference or citation that supports the answer."""
    document_id: str
    content: str    
    page_number: Optional[int] = None
    section_title: Optional[str] = None


class QueryRequest(BaseModel):
    """Request model for the agent endpoint."""
    question: str = Field(
        ..., 
        min_length=1, 
        max_length=1000, 
        description="The natural language question to answer"
    )
    selected_text: Optional[str] = Field(
        None, 
        max_length=5000, 
        description="Specific text selected by the user from the book"
    )
    mode: Optional[str] = Field(
        None,
        pattern=r"^(selected_text|book|general)$",
        description="The mode to use when answering (selected_text, book, or general)"
    )


class QueryResponse(BaseModel):
    """Response model for the agent endpoint."""
    answer: str
    sources: List[Source]
    mode: str = Field(
        ...,
        pattern=r"^(selected_text|book|general)$",
        description="The mode used to generate this answer"
    )