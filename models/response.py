from pydantic import BaseModel
from typing import List, Optional, Dict, Any


class SourceReference(BaseModel):
    chunk_id: Optional[str] = None
    text_preview: str
    relevance_score: Optional[float] = None
    location: Optional[Dict[str, Any]] = None


class BookWideQueryResponse(BaseModel):
    answer: str
    sources: List[SourceReference]
    session_id: str
    confidence: Optional[float] = None


class SelectedTextQueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]  # For selected text mode
    session_id: str
    mode: str = "selected_text"
    confidence: Optional[float] = None


class HealthCheckResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: Dict[str, bool]