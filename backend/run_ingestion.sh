#!/bin/bash
# Script to run the RAG chatbot ingestion process

# Navigate to the backend directory
cd /mnt/d/GIAIC/Quarter 4/Claude Code/hackathon-1/backend

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Install dependencies
pip install -r requirements.txt

# Run the ingestion script to load textbook content into Qdrant
python ingest.py