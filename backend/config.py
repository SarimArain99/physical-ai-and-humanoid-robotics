import os
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field

# 1. Calculate the absolute path to the backend folder
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 2. Build the exact path to the .env file
ENV_FILE_PATH = os.path.join(BASE_DIR, ".env")

class Settings(BaseSettings):
    # API Keys
    openai_api_key: str = Field(..., alias="OPENAI_API_KEY")
    # Make API Key optional (default to None) in case you run Qdrant locally without one
    qdrant_api_key: str | None = Field(default=None, alias="QDRANT_API_KEY")

    # URLs and configuration
    qdrant_url: str = Field(..., alias="QDRANT_URL")

    # Neon DB and Authentication
    neon_api_key: str = Field(..., alias="NEON_API_KEY")
    better_auth_secret: str = Field(..., alias="BETTER_AUTH_SECRET")
    better_auth_url: str = Field(default="http://localhost:3000", alias="BETTER_AUTH_URL")

    # Other settings with defaults
    qdrant_collection_name: str = "claude-cli-db"
    docs_path: str = "../frontend/docs"

    # Pydantic v2 Config
    model_config = SettingsConfigDict(
        # 3. Use the absolute path we calculated above
        env_file=ENV_FILE_PATH,
        env_file_encoding="utf-8",
        extra="ignore"
    )

settings = Settings()