from pydantic import BaseModel
from typing import Optional, Dict, Any


class BookWideQueryRequest(BaseModel):
    query: str
    session_id: str
    metadata: Optional[Dict[str, Any]] = None


class SelectedTextQueryRequest(BaseModel):
    query: str
    selected_text: str
    session_id: str
    metadata: Optional[Dict[str, Any]] = None


class HealthCheckRequest(BaseModel):
    pass