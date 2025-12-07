# User Story 2: RAG Chatbot Integration

## Overview
Implement a Retrieval-Augmented Generation (RAG) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content. The chatbot will be integrated as a floating widget in the Docusaurus frontend and will use Qdrant as the vector database.

## User Story
As a learner studying Physical AI & Humanoid Robotics,
I want to ask questions about the textbook content through an AI assistant,
So that I can get immediate, contextual answers based on the book's material.

## Acceptance Criteria
- [ ] Backend service with FastAPI provides ingestion and chat endpoints
- [ ] Ingestion endpoint parses all .md files from the docs folder and stores embeddings in Qdrant
- [ ] Chat endpoint accepts queries and returns contextually relevant answers from the textbook
- [ ] Frontend chat widget allows users to ask questions and receive responses
- [ ] Text selection feature captures selected text and provides it as context to the chatbot
- [ ] Chatbot strictly answers only questions related to the book's content
- [ ] Chatbot prioritizes selected text when provided as context

## Technical Requirements
- Backend: Python with FastAPI
- AI Engine: OpenAI API (text-embedding-3-small, gpt-3.5-turbo)
- Vector Database: Qdrant Cloud (Free Tier)
- Frontend: React component integrated with Docusaurus
- No SQL database for chat logs (purely retrieval-based)

## Dependencies
- Basic textbook content available in Docusaurus docs folder
- OpenAI API key
- Qdrant Cloud account and API key

## Constraints
- Must not store chat logs in a database
- Must refuse to answer questions unrelated to the book's content
- Must preserve context headers when chunking text
- Must handle text selection from the frontend

## Performance Requirements
- Response time under 5 seconds for typical queries
- Support for concurrent users
- Proper error handling and user feedback

## Success Metrics
- Users can ask questions and receive accurate answers from the textbook
- Text selection feature works properly
- Chatbot correctly identifies and refuses off-topic questions
- System handles errors gracefully