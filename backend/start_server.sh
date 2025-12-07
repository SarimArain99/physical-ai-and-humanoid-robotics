#!/bin/bash
# Script to start the RAG chatbot backend server

# Navigate to the backend directory
cd /mnt/d/GIAIC/Quarter 4/Claude Code/hackathon-1/backend

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Install dependencies
pip install -r requirements.txt

# Start the FastAPI server
uvicorn main:app --host 0.0.0.0 --port 8000 --reload