# Implementation Plan: User Story 2 - RAG Chatbot Integration

## Objective
Implement a Retrieval-Augmented Generation (RAG) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content, integrated as a floating widget in the Docusaurus frontend.

## Scope & Dependencies
### In Scope
- FastAPI backend with ingestion and chat endpoints
- Qdrant vector database integration
- Text parsing and embedding from Docusaurus docs
- Frontend chat widget with text selection
- OpenAI API integration for embeddings and responses

### Out of Scope
- User authentication and management (handled separately)
- Chat history storage (purely retrieval-based)
- Complex conversation memory

### External Dependencies
- OpenAI API (for embeddings and completions)
- Qdrant Cloud (vector storage)
- Docusaurus frontend (for widget integration)
- Existing textbook content in docs folder

## Key Decisions & Rationale

### Technology Stack Decision
- **FastAPI**: Selected for its performance, async support, and excellent documentation
- **Qdrant**: Selected for its cloud offering and efficient vector search capabilities
- **OpenAI API**: Selected for reliable embeddings and text generation
- **React**: Selected for the chat widget due to Docusaurus compatibility

### Approach: Stateless RAG Architecture
- **Option 1**: Stateful agent with conversation memory (more complex, requires DB)
- **Option 2**: Stateless RAG with context passing (simpler, meets requirements)
- **Selected**: Option 2 - Stateless RAG approach to maintain simplicity and meet requirements

## API Contracts

### Backend Endpoints
```
POST /ingest
- Description: Parse docs folder and store embeddings in Qdrant
- Request: None (uses configured docs path)
- Response: { "message": "Document ingestion started in the background" }

POST /chat
- Description: Process user query with RAG
- Request: { "query": "user question", "selected_text": "optional selected text" }
- Response: { "response": "AI-generated answer" }

GET /health
- Description: Health check
- Response: { "status": "healthy" }
```

### Frontend Integration
- Floating chat widget component
- Text selection capture using window.getSelection()
- WebSocket streaming for responses (if needed) or standard API calls

## Non-Functional Requirements

### Performance
- P95 response time: < 5 seconds
- Support for 10+ concurrent users
- Efficient text chunking to minimize token usage

### Reliability
- SLO: 99% uptime for chat endpoint
- Graceful degradation when OpenAI/Qdrant services are unavailable
- Proper error handling and user feedback

### Security
- API keys stored in environment variables
- Input validation for user queries
- Content filtering to prevent inappropriate usage

### Cost
- Optimize embedding model usage (text-embedding-3-small)
- Efficient chunking to minimize API calls

## Data Management
- Text chunks stored in Qdrant with context headers
- No persistent chat logs stored (stateless operation)
- Document metadata preserved (file path, headers)

## Operational Readiness

### Monitoring
- Health check endpoint
- Basic logging for debugging
- Error tracking for API failures

### Deployment
- Separate backend deployment (not integrated with Docusaurus)
- Environment variable configuration
- Docker support for containerization

## Risk Analysis

### Top 3 Risks
1. **API Costs**: High usage could lead to significant OpenAI/Qdrant costs
   - Mitigation: Implement rate limiting, monitor usage, consider caching

2. **Response Quality**: RAG may return inaccurate or hallucinated responses
   - Mitigation: Prompt engineering, content validation, clear scope limitations

3. **Performance**: Large document collections could slow search and response
   - Mitigation: Optimize chunking strategy, implement caching, monitor performance

## Implementation Phases

### Phase 1: Backend Infrastructure
- Set up FastAPI application
- Implement Qdrant client integration
- Create ingestion endpoint with document parsing
- Implement chat endpoint with RAG logic

### Phase 2: Frontend Integration
- Create React chat widget component
- Implement text selection capture
- Integrate with backend API
- Add UI/UX polish

### Phase 3: Testing & Optimization
- Test with full textbook content
- Optimize chunking strategy
- Performance testing
- User acceptance testing

## Success Criteria
- Chatbot accurately answers questions based on textbook content
- Text selection feature works properly
- System handles errors gracefully
- Response times meet performance requirements
- Off-topic questions are properly rejected