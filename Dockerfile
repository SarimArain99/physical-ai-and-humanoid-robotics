# Hugging Face Spaces Docker SDK Configuration
# This Dockerfile builds the FastAPI backend from the backend/ directory
# Read the doc: https://huggingface.co/docs/hub/spaces-sdks-docker
FROM python:3.11-slim

# Create a non-root user with explicit UID 1000
RUN useradd -m -u 1000 user
USER user

# Set up the environment
ENV PATH="/home/user/.local/bin:$PATH"
WORKDIR /app

# Copy and install requirements from backend directory
COPY --chown=user backend/requirements.txt requirements.txt
RUN pip install --no-cache-dir --upgrade -r requirements.txt

# Copy application files from backend directory
COPY --chown=user backend/ .

# HF Spaces Docker SDK uses port 7860 by default
# Use ${PORT} variable which HF sets automatically, defaulting to 7860
CMD ["sh", "-c", "uvicorn main:app --host 0.0.0.0 --port ${PORT:-7860}"]
