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

# AI-Native Textbook Backend

RAG-powered chatbot backend for the AI-Native Textbook on Physical AI & Humanoid Robotics. Deployed on Hugging Face Spaces using Docker SDK.

## Features

- **RAG Chatbot**: Retrieval-Augmented Generation using OpenAI and Qdrant
- **Document Ingestion**: Parses Markdown files and stores embeddings in Qdrant
- **Smart Search**: Finds relevant textbook content based on user queries
- **Authentication**: Better Auth integration for user management
- **Health Monitoring**: Circuit breaker pattern for external service resilience

## API Endpoints

### Health Endpoints

- `GET /` - Root status endpoint
- `GET /health` - Health check
- `GET /health/circuits` - Circuit breaker status for OpenAI and Qdrant

### Authentication

- `POST /api/auth/sign-up` - User registration
- `POST /api/auth/sign-in` - User login
- `POST /api/auth/sign-out` - User logout

### Chat

- `POST /chat` - Process user query with RAG

## Required Environment Variables (Secrets)

Configure these in Hugging Face Space Settings:

| Variable | Description |
|----------|-------------|
| `OPENAI_API_KEY` | OpenAI API key for embeddings and chat |
| `QDRANT_URL` | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Qdrant API key (if using cloud) |
| `NEON_API_KEY` | PostgreSQL connection string (Neon) |
| `BETTER_AUTH_SECRET` | JWT secret for authentication |
| `BETTER_AUTH_URL` | Backend URL for auth callbacks (this Space URL) |
| `CORS_ALLOWED_ORIGINS` | Comma-separated allowed frontend origins |

## Frontend URLs (for CORS)

Add these to `CORS_ALLOWED_ORIGINS`:
- `https://physical-ai-and-humanoid-robotics-omega.vercel.app`
