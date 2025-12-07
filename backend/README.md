# RAG Chatbot Backend for Physical AI & Humanoid Robotics Textbook

This backend service provides a Retrieval-Augmented Generation (RAG) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content.

## Features

- **Document Ingestion**: Parses all Markdown files from the Docusaurus docs folder and stores embeddings in Qdrant
- **Smart Search**: Finds the most relevant textbook content based on user queries
- **Contextual Responses**: Answers questions using textbook content with proper context
- **Text Selection Support**: Prioritizes selected text when provided as context
- **Floating Chat Widget**: Integrated into the Docusaurus frontend

## Architecture

- **Backend**: FastAPI application
- **AI Engine**: OpenAI API (text-embedding-3-small, gpt-3.5-turbo)
- **Vector Database**: Qdrant Cloud
- **Frontend Integration**: React chat widget component

## API Endpoints

- `POST /ingest` - Parse docs folder and store embeddings in Qdrant
- `POST /chat` - Process user query with RAG and return contextual answer
- `GET /health` - Health check endpoint

## Setup and Installation

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and Qdrant configuration
   ```

3. Run the ingestion process to load textbook content:
   ```bash
   python ingest.py
   ```

4. Start the server:
   ```bash
   uvicorn main:app --host 0.0.0.0 --port 8000
   ```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_HOST`: Your Qdrant Cloud cluster URL
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: claude-cli-db)
- `DOCS_PATH`: Path to the Docusaurus docs folder (default: ../frontend/docs)

## Usage

1. First, run the ingestion process to load all textbook content into the vector database:
   ```bash
   python ingest.py
   ```

2. Start the backend server:
   ```bash
   uvicorn main:app --host 0.0.0.0 --port 8000
   ```

3. The chat widget is automatically integrated into the Docusaurus frontend

## API Usage

### Chat Endpoint
```
POST /chat
Content-Type: application/json

{
  "query": "Your question about the textbook",
  "selected_text": "Optional selected text for context (prioritized)"
}
```

Response:
```json
{
  "response": "AI-generated answer based on textbook content"
}
```

### Ingestion Endpoint
```
POST /ingest
```

Response:
```json
{
  "message": "Document ingestion started in the background"
}
```

## Frontend Integration

The chat widget is automatically integrated into the Docusaurus site through:
- `frontend/src/components/ChatWidget/index.js` - The chat widget component
- `frontend/src/theme/Layout/index.js` - Layout wrapper that includes the widget

The widget features:
- Floating button that toggles the chat interface
- Text selection capture using `window.getSelection()`
- Real-time communication with the backend
- Loading states and error handling

## Security Considerations

- API keys are stored in environment variables
- The system only responds to questions related to textbook content
- No chat logs are stored (stateless RAG approach)
- Input validation is implemented for user queries

## Performance Optimization

- Text is chunked to maintain context while optimizing token usage
- Efficient vector search in Qdrant for relevant results
- Proper error handling to ensure system stability

## Troubleshooting

If the chatbot doesn't respond:
1. Verify that the ingestion process completed successfully
2. Check that your OpenAI and Qdrant API keys are valid
3. Confirm that the Qdrant collection exists and contains data
4. Verify the backend server is running and accessible

## License

This project is part of the AI-Native Textbook on Physical AI & Humanoid Robotics for the Panaversity Hackathon I.