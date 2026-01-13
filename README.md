---
title: AI-Native Book Backend
emoji: 📚
colorFrom: blue
colorTo: purple
sdk: docker
sdk_version: "3.11"
pinned: false
license: mit
---


# AI-Native Textbook on Physical AI & Humanoid Robotics

This repository contains an AI-native, interactive textbook for teaching **Physical AI & Humanoid Robotics**. The project is part of the Hackathon I challenge and implements all technical, pedagogical, and functional requirements as outlined in the hackathon brief.

## Features

- **Comprehensive Textbook**: Complete 13-week course covering all 4 modules (ROS 2, Digital Twin, AI-Robot Brain, VLA)
- **AI-Powered Chatbot**: RAG-based chatbot that answers questions about textbook content
- **User Authentication**: Secure login with Better Auth and user profiling
- **Personalization**: Content adaptation based on user's technical background and hardware access
- **Urdu Translation**: Real-time translation of content to Urdu for broader accessibility
- **Interactive Learning**: Embedded tools for hands-on learning and experimentation

## Tech Stack

- **Frontend**: Docusaurus for textbook content, deployed to GitHub Pages
- **Backend**: FastAPI for services, with OpenAI API for RAG functionality
- **Authentication**: Better Auth for secure user management
- **Database**: Neon Postgres for user data, Qdrant Cloud for vector storage
- **AI Services**: OpenAI API for chatbot and translation functionality

## Setup Instructions

### Prerequisites

- Node.js (v18 or higher)
- Python (v3.11 or higher)
- Git
- Access to OpenAI API
- Neon Postgres account
- Qdrant Cloud account

### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables by creating a `.env` file in the backend directory:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_HOST=your_qdrant_cluster_url
   QDRANT_COLLECTION_NAME=claude-cli-db
   ```

5. Load textbook content into the vector database:
   ```bash
   python ingest.py
   ```

6. Start the backend server:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

### Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run the development server:
   ```bash
   npm start
   ```

The textbook will be accessible at `http://localhost:3000`.

## Deployment

### Frontend to GitHub Pages

```bash
npm run build
npm run deploy
```

### Backend

The backend can be deployed to any cloud platform that supports Python applications (e.g., Heroku, Render, AWS, etc.).

## Architecture

The application follows a microservice architecture with a clear separation between the frontend (Docusaurus-based textbook) and backend (FastAPI services). This structure allows for optimal separation of concerns while meeting all hackathon requirements.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the terms specified in the LICENSE file.# physical-ai-and-humanoid-robotics
# physical-ai-and-humanoid-robotics
