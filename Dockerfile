# RETIRED 2026-11-08 — the Hugging Face deployment is gone; the backend
# runs on Vercel from chatbot-api-node/. This Dockerfile is kept for
# local reference only and is not deployed anywhere.
FROM python:3.11-slim

# Create a non-root user with explicit UID 1000
RUN useradd -m -u 1000 user
USER user

# Set up the environment
ENV PATH="/home/user/.local/bin:$PATH"
WORKDIR /app

# Copy and install requirements (files are at root after sync)
COPY --chown=user requirements.txt requirements.txt
RUN pip install --no-cache-dir --upgrade -r requirements.txt

# Copy application files (files are at root after sync)
COPY --chown=user . .

# HF Spaces Docker SDK uses port 7860 by default
# Use ${PORT} variable which HF sets automatically, defaulting to 7860
CMD ["sh", "-c", "uvicorn main:app --host 0.0.0.0 --port ${PORT:-7860}"]
