from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for textbook content management and RAG chatbot functionality",
    version="1.0.0"
)

# Data models based on the plan
class Reference(BaseModel):
    id: str
    type: str
    title: str
    authors: List[str]
    year: int
    url: Optional[str] = None
    doi: Optional[str] = None
    abstract: Optional[str] = None

class Chapter(BaseModel):
    id: str
    name: str
    slug: str
    module: str
    content: str
    metadata: Optional[dict] = {}
    references: Optional[List[Reference]] = []
    learning_objectives: Optional[List[str]] = []
    prerequisites: Optional[List[str]] = []
    created_at: str
    updated_at: str

class QueryRequest(BaseModel):
    query: str
    context: Optional[str] = None
    user_id: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    sources: List[str]
    confidence: float
    context_used: Optional[str] = None

# In-memory storage for demo purposes (would use database in production)
chapters_db = {}
references_db = {}

# Mock data for initial testing
mock_chapters = {
    "physical-ai-foundations": {
        "id": "ch-001",
        "name": "Foundations of Physical AI",
        "slug": "physical-ai-foundations",
        "module": "foundations",
        "content": "# Foundations of Physical AI\n\nPhysical AI represents a paradigm shift from traditional digital AI to embodied intelligence systems...",
        "metadata": {"word_count": 1200, "reading_time": "5 min"},
        "references": [],
        "learning_objectives": [
            "Understand the difference between digital and physical AI",
            "Explain embodied cognition principles"
        ],
        "prerequisites": ["Basic understanding of AI concepts"],
        "created_at": "2026-01-02T00:00:00Z",
        "updated_at": "2026-01-02T00:00:00Z"
    }
}

# Populate mock data
chapters_db.update(mock_chapters)

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics Textbook API"}

@app.get("/api/chapters", response_model=List[Chapter])
async def get_chapters():
    """Get all textbook chapters"""
    return list(chapters_db.values())

@app.get("/api/chapters/{slug}", response_model=Chapter)
async def get_chapter(slug: str):
    """Get a specific chapter by slug"""
    if slug in chapters_db:
        return chapters_db[slug]
    raise HTTPException(status_code=404, detail="Chapter not found")

@app.post("/api/chat", response_model=QueryResponse)
async def chat_with_textbook(request: QueryRequest):
    """Submit a query to the RAG chatbot"""
    try:
        # This is a placeholder for the RAG implementation
        # In a real system, this would:
        # 1. Embed the query using sentence transformers
        # 2. Search vector database (Qdrant) for relevant content
        # 3. Generate response using LLM (OpenAI) with retrieved context

        # For now, return a more informative response
        response_text = f"I've received your query about: '{request.query}'. In a full implementation, this would search the textbook content and provide an accurate, contextually relevant answer based on the source material."

        return QueryResponse(
            response=response_text,
            sources=["textbook-content-placeholder"],
            confidence=0.8,
            context_used=request.context
        )

    except Exception as e:
        # Handle any errors gracefully
        error_msg = f"I encountered an error processing your query: {str(e)}. Please check that all required dependencies are installed and environment variables are set."

        return QueryResponse(
            response=error_msg,
            sources=[],
            confidence=0.1,
            context_used=request.context
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)