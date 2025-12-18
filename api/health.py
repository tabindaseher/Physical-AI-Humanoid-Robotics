from fastapi import APIRouter
from datetime import datetime
from typing import Dict, Any
# Use relative imports within the backend package
from ..models.response import HealthCheckResponse
from ..config.settings import settings
import httpx
import asyncio

router = APIRouter()


@router.get("/health", response_model=HealthCheckResponse)
async def health_check() -> HealthCheckResponse:
    """
    Health check endpoint to verify service and dependencies are accessible.
    """
    # Check if environment variables are properly set
    dependencies_status: Dict[str, bool] = {
        "cohere": settings.cohere_api_key is not None and len(settings.cohere_api_key) > 0,
        "qdrant": settings.qdrant_url is not None and len(settings.qdrant_url) > 0,
        "postgres": settings.neon_database_url is not None and len(settings.neon_database_url) > 0
    }

    # Basic status - if all required env vars are present
    overall_status = "healthy" if all(dependencies_status.values()) else "degraded"

    return HealthCheckResponse(
        status=overall_status,
        timestamp=datetime.utcnow().isoformat(),
        dependencies=dependencies_status
    )