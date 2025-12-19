from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Cohere Configuration
    cohere_api_key: str

    # Qdrant Configuration
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_cluster_id: Optional[str] = None

    # Database Configuration
    neon_database_url: str
    postgres_url: Optional[str] = None

    # Application Configuration
    environment: str = "development"
    log_level: str = "info"

    model_config = {"env_file": ".env", "env_file_encoding": "utf-8"}



settings = Settings()