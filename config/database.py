from sqlmodel import create_engine, Session
from backend.config.settings import settings
from typing import Generator


# Create database engine
engine = create_engine(
    settings.neon_database_url,
    echo=(settings.environment == "development")
)


def get_session() -> Generator[Session, None, None]:
    with Session(engine) as session:
        yield session