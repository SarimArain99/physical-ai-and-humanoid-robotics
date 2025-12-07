# Tasks: User Story 2 - RAG Chatbot Integration

## Phase 1: Backend Infrastructure

### Task 1.1: Set up Backend Project Structure
- [x] Create backend/ directory
- [x] Create requirements.txt with dependencies (FastAPI, OpenAI, Qdrant, etc.)
- [x] Set up configuration with environment variables
- [x] Create .env.example file

### Task 1.2: Implement Document Ingestion
- [x] Create ingest.py script to parse .md files from docs folder
- [x] Implement text splitting logic that preserves context headers
- [x] Implement OpenAI embedding generation
- [x] Implement Qdrant vector storage
- [x] Add proper error handling and logging

### Task 1.3: Create FastAPI Backend
- [x] Implement main.py with FastAPI application
- [x] Create /ingest endpoint for document processing
- [x] Create /chat endpoint with RAG logic
- [x] Implement health check endpoint
- [x] Add proper error handling and validation

## Phase 2: Frontend Integration

### Task 2.1: Create Chat Widget Component
- [x] Create React component for chat widget
- [x] Implement toggle functionality (show/hide)
- [x] Create message display area
- [x] Add input area with send functionality
- [x] Style component to match Docusaurus theme

### Task 2.2: Integrate with Backend API
- [x] Implement API calls to backend endpoints
- [x] Handle loading states and errors
- [x] Implement message history display
- [x] Add welcome message for new users

### Task 2.3: Text Selection Feature
- [x] Implement text selection capture using window.getSelection()
- [x] Pass selected text to chat API as context
- [x] Prioritize selected text in responses
- [x] Test integration with Docusaurus content

## Phase 3: Testing & Validation

### Task 3.1: Backend Testing
- [ ] Test ingestion with full textbook content
- [ ] Verify vector storage in Qdrant
- [ ] Test chat endpoint with various queries
- [ ] Validate response quality and relevance

### Task 3.2: Frontend Testing
- [ ] Test chat widget functionality across pages
- [ ] Verify text selection capture works properly
- [ ] Test error handling and edge cases
- [ ] Validate responsive design

### Task 3.3: Integration Testing
- [ ] End-to-end testing of RAG functionality
- [ ] Verify off-topic question rejection
- [ ] Test performance with realistic query loads
- [ ] Validate content accuracy

## Phase 4: Optimization & Deployment

### Task 4.1: Performance Optimization
- [ ] Optimize text chunking strategy
- [ ] Implement caching if needed
- [ ] Optimize API response times
- [ ] Monitor resource usage

### Task 4.2: Security & Error Handling
- [ ] Add input validation
- [ ] Implement rate limiting if necessary
- [ ] Add comprehensive error handling
- [ ] Ensure secure API key management

## Acceptance Tests

### Test Case 1: Basic Chat Functionality
**Given**: User has opened the chat widget
**When**: User types a question about Physical AI content
**Then**: Chatbot responds with relevant information from the textbook

### Test Case 2: Text Selection
**Given**: User has selected text on a textbook page
**When**: User opens chat and asks a question
**Then**: Chatbot prioritizes the selected text in its response

### Test Case 3: Off-topic Question
**Given**: User asks a question unrelated to Physical AI
**When**: User submits the question
**Then**: Chatbot politely declines and suggests focusing on textbook topics

### Test Case 4: Document Ingestion
**Given**: Textbook content exists in docs folder
**When**: Admin calls /ingest endpoint
**Then**: All documents are parsed and stored in Qdrant

### Test Case 5: Error Handling
**Given**: Backend services are temporarily unavailable
**When**: User submits a question
**Then**: User receives appropriate error message

## Definition of Done
- [ ] All tasks in Phases 1-3 completed
- [ ] All acceptance tests pass
- [ ] Code reviewed and approved
- [ ] Performance requirements met
- [ ] Security requirements satisfied
- [ ] Documentation updated