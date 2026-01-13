"""
Hugging Face Spaces Compatible Configuration

This module handles environment-based configuration for the backend.
It supports both .env files for local development and environment variables
for Hugging Face Spaces deployment.
"""
import os
import sys
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field

# 1. Calculate the absolute path to the backend folder
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 2. Build the exact path to the .env file
ENV_FILE_PATH = os.path.join(BASE_DIR, ".env")

# 3. Make env_file optional for Hugging Face Spaces compatibility
# HF Spaces uses environment variables (Secrets) instead of .env files
env_file = ENV_FILE_PATH if os.path.exists(ENV_FILE_PATH) else None


class Settings(BaseSettings):
    """Application settings with Hugging Face Spaces support."""

    # API Keys
    openai_api_key: str = Field(..., alias="OPENAI_API_KEY",
                                description="OpenAI API key for embeddings and chat")
    # Make API Key optional (default to None) for local Qdrant instances
    qdrant_api_key: str | None = Field(default=None, alias="QDRANT_API_KEY",
                                        description="Qdrant API key (optional for local instances)")

    # URLs and configuration
    qdrant_url: str = Field(..., alias="QDRANT_URL",
                          description="Qdrant Cloud cluster URL")

    # Neon DB and Authentication
    neon_api_key: str = Field(..., alias="NEON_API_KEY",
                             description="PostgreSQL connection string (Neon)")
    better_auth_secret: str = Field(..., alias="BETTER_AUTH_SECRET",
                                    description="JWT secret for authentication signing")
    better_auth_url: str = Field(default="http://localhost:3000", alias="BETTER_AUTH_URL",
                                  description="Backend URL for auth callbacks")

    # Other settings with defaults
    qdrant_collection_name: str = "claude-cli-db"
    docs_path: str = "../frontend/docs"

    # Pydantic v2 Config
    model_config = SettingsConfigDict(
        # Use env_file only if it exists (None for HF Spaces)
        env_file=env_file,
        env_file_encoding="utf-8",
        extra="ignore"
    )


def validate_settings() -> None:
    """
    Validate that all required environment variables are set.
    Provides clear error messages for missing configuration.
    Exits with error code 1 if validation fails.
    """
    required_vars = {
        "OPENAI_API_KEY": "OpenAI API key for embeddings and chat completions",
        "QDRANT_URL": "Qdrant Cloud cluster URL",
        "NEON_API_KEY": "PostgreSQL connection string (Neon)",
        "BETTER_AUTH_SECRET": "JWT secret for authentication signing",
    }

    missing = []
    for var_name, description in required_vars.items():
        if not os.environ.get(var_name):
            missing.append(f"  - {var_name}: {description}")

    if missing:
        print("❌ Missing required environment variables:", file=sys.stderr)
        for item in missing:
            print(item, file=sys.stderr)
        print("\nPlease configure these variables in Hugging Face Spaces Secrets.", file=sys.stderr)
        sys.exit(1)


# Validate settings on import (only for production/HF Spaces)
# Skip validation for local development if .env file exists
if env_file is None:
    # Running in HF Spaces (no .env file) - validate strictly
    try:
        settings = Settings()
    except Exception as e:
        print(f"❌ Configuration error: {e}", file=sys.stderr)
        print("\nPlease check your Hugging Face Spaces Secrets configuration.", file=sys.stderr)
        sys.exit(1)
else:
    # Running locally with .env file - load settings
    settings = Settings()