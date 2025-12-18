from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
import time
import logging
from typing import Dict, Set
from collections import defaultdict, deque
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)


class RateLimitMiddleware:
    def __init__(self, requests_limit: int = 100, window_size: int = 3600):  # 100 requests per hour
        self.requests_limit = requests_limit
        self.window_size = window_size  # in seconds
        self.requests: Dict[str, deque] = defaultdict(deque)

    async def __call__(self, request: Request, call_next):
        client_ip = self.get_client_ip(request)

        # Clean old requests outside the window
        now = time.time()
        while (self.requests[client_ip] and
               self.requests[client_ip][0] <= now - self.window_size):
            self.requests[client_ip].popleft()

        # Check if limit exceeded
        if len(self.requests[client_ip]) >= self.requests_limit:
            logger.warning(f"Rate limit exceeded for IP: {client_ip}")
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={"detail": "Rate limit exceeded"}
            )

        # Add current request
        self.requests[client_ip].append(now)

        response = await call_next(request)
        return response

    def get_client_ip(self, request: Request) -> str:
        # Try to get the real client IP from headers
        forwarded_for = request.headers.get("x-forwarded-for")
        if forwarded_for:
            return forwarded_for.split(",")[0].strip()

        real_ip = request.headers.get("x-real-ip")
        if real_ip:
            return real_ip.strip()

        # Fallback to client host
        return request.client.host


def add_security_headers(response):
    """Add security headers to response"""
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    return response