from pydantic_settings import BaseSettings
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[2]  # backend/

class Settings(BaseSettings):
    # --- Core ---
    app_name: str = "RAG Chatbot Backend"
    app_version: str = "1.0.0"
    debug: bool = False
    host: str = "0.0.0.0"
    port: int = 8000
    log_level: str = "INFO"

    # --- Services ---
    NEON_DATABASE_URL: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str
    OPENAI_API_KEY: str | None = None
    # --- Ingestion (optional but present in .env) ---
    BOOK_SOURCE_DIR: str | None = None
    INGESTION_CHUNK_SIZE: int | None = None
    INGESTION_OVERLAP: int | None = None
    OPENAI_EMBEDDING_MODEL: str ="gpt-3.5-turbo"

    class Config:
        env_file = BASE_DIR / ".env"
        case_sensitive = False
        extra = "ignore"   # 🔥 THIS LINE FIXES EVERYTHING

settings = Settings()
