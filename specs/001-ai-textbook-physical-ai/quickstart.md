# Quickstart Guide: AI-Native Textbook on Physical AI & Humanoid Robotics

## Overview

This guide provides instructions for setting up and running the AI-Native Textbook application. The project consists of a frontend Docusaurus textbook and a backend FastAPI service that provides authentication, RAG chatbot, translation, personalization, and meets all security, observability, and scalability requirements.

## Prerequisites

- **Node.js** (v18 or higher)
- **Python** (v3.11 or higher)
- **Git**
- **Access to OpenAI API** (for RAG chatbot and translation)
- **Neon Postgres account** (for user data storage)
- **Qdrant Cloud account** (for vector storage)
- **Better Auth account** (for authentication)
- **Docker** (optional, for containerized deployment)

## Getting Started

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Backend (FastAPI)

#### Navigate to backend directory
```bash
cd backend
```

#### Create virtual environment and install dependencies
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

#### Set up environment variables
Create a `.env` file in the backend directory:

```env
# Database
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# OpenAI
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant
QDRANT_URL=https://your-cluster-url.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key

# Better Auth
BETTER_AUTH_SECRET=your_auth_secret
BETTER_AUTH_URL=http://localhost:3000

# Encryption
ENCRYPTION_KEY=your_32_character_base64_key_here  # AES-256 key

# Application
API_HOST=0.0.0.0
API_PORT=8000
DEBUG=false
LOG_LEVEL=INFO

# Observability
OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317  # For OpenTelemetry
```

#### Run database migrations
```bash
# If using Alembic for migrations
alembic upgrade head
```

#### Start the backend server
```bash
python -m src.main
# or
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 3. Set Up Frontend (Docusaurus)

#### Navigate to frontend directory
```bash
cd frontend  # from repository root
```

#### Install dependencies
```bash
npm install
```

#### Set up environment variables
Create a `.env` file in the frontend directory:

```env
# Backend API
REACT_APP_API_BASE_URL=http://localhost:8000

# Better Auth
REACT_APP_BETTER_AUTH_URL=http://localhost:8000

# Performance monitoring
REACT_APP_UI_TIMEOUT=200  # Maximum time for UI interactions in ms
```

#### Run the development server
```bash
npm start
```

The textbook should now be accessible at `http://localhost:3000`.

## Development Workflow

### Adding New Textbook Content

1. Create new markdown files in `frontend/docs/week-XX/`
2. Update `frontend/sidebars.js` to include the new content in navigation
3. Add learning objectives, exercises, and review questions as specified in the constitution

### Adding New API Endpoints

1. Define the endpoint in `backend/src/api/`
2. Create corresponding service functions in `backend/src/services/`
3. Update the OpenAPI contract in `specs/001-ai-textbook-physical-ai/contracts/`
4. Add tests in `backend/tests/`

### Running Tests

#### Backend tests
```bash
cd backend
source venv/bin/activate
pytest tests/
```

#### Frontend tests
```bash
cd frontend
npm test
```

## Configuration

### Content Structure

The textbook content follows this structure:
```
frontend/docs/
├── week-01/
│   ├── introduction-to-ros2.md
│   └── ros2-nodes-and-topics.md
├── week-02/
└── ...
```

Each chapter file should follow the structure defined in the data model with appropriate frontmatter:

```markdown
---
title: Introduction to ROS 2
week: 1
module: ros2
learning_objectives:
  - Understand the basic concepts of ROS 2
  - Learn about nodes and topics
prerequisites:
  - Basic Python knowledge
  - Understanding of robotics concepts
---

# Introduction to ROS 2

Content goes here...
```

### Environment Variables

#### Backend (.env)
- `DATABASE_URL`: Neon Postgres connection string
- `OPENAI_API_KEY`: OpenAI API key for RAG and translation
- `QDRANT_URL`: Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Qdrant API key
- `BETTER_AUTH_SECRET`: Secret for Better Auth
- `ENCRYPTION_KEY`: AES-256 key for data encryption
- `OTEL_EXPORTER_OTLP_ENDPOINT`: OpenTelemetry endpoint for observability
- `API_HOST`/`API_PORT`: Server configuration

#### Frontend (.env)
- `REACT_APP_API_BASE_URL`: Backend API base URL
- `REACT_APP_BETTER_AUTH_URL`: Better Auth endpoint
- `REACT_APP_UI_TIMEOUT`: Maximum time for UI interactions

## Security Considerations

### Data Encryption
- All sensitive user data is encrypted at rest using AES-256
- User profile data, session tokens, and chat query content are encrypted
- Use a strong, randomly generated 32-character base64 encryption key
- Store encryption keys securely using environment variables or secret management tools

### API Security
- Rate limiting is implemented on expensive operations (translation, RAG queries)
- Input validation is performed on all user-provided content
- Authentication is required for personalization and translation features
- All API communications use HTTPS encryption

## Observability Setup

### Logging
- Structured logging is implemented using JSON format
- Log levels can be configured via the LOG_LEVEL environment variable
- All user actions and security events are logged

### Metrics
- Prometheus metrics are available at `/metrics` endpoint
- Key metrics include request counts, response times, and error rates
- API call costs are tracked for budget management

### Tracing
- Distributed tracing is implemented using OpenTelemetry
- Traces are exported to the configured OTLP endpoint
- All requests are traced end-to-end across services

## Performance Optimization

### Caching Strategy
- Translation results are cached to improve performance
- Personalized content is cached based on user profile
- Textbook content is cached with appropriate invalidation

### CDN Configuration
- Static textbook content is served via CDN for fast access
- Configure your CDN to cache content with appropriate TTL values
- Implement cache invalidation when content is updated

## Scalability Configuration

### Load Testing
- The system is designed to support 1000+ concurrent users
- Use load testing tools to validate performance under expected load
- Monitor response times and error rates during load testing

### Horizontal Scaling
- Backend services are stateless and can be scaled horizontally
- Use a load balancer to distribute traffic across multiple instances
- Ensure database connections are properly managed when scaling

## Deployment

### Frontend to GitHub Pages

```bash
cd frontend
npm run build
npm run deploy
```

### Backend to Cloud Platform

The backend can be deployed to any cloud platform that supports Python applications (e.g., Heroku, Render, AWS, etc.). The application is configured to run with a WSGI/ASGI server like uvicorn.

### Containerized Deployment

```bash
# Build the Docker image
docker build -t ai-textbook-backend .

# Run the container
docker run -d -p 8000:8000 --env-file .env ai-textbook-backend
```

## Troubleshooting

### Common Issues

1. **Database Connection Issues**: Verify your Neon Postgres connection string and credentials
2. **OpenAI API Errors**: Check that your API key is valid and you have sufficient credits
3. **Qdrant Connection Issues**: Verify your Qdrant cluster URL and API key
4. **Authentication Problems**: Ensure Better Auth is properly configured
5. **Encryption Errors**: Verify that the ENCRYPTION_KEY is a valid 32-character base64 string

### Performance Issues

- Check that caching is properly configured
- Verify CDN is serving static content
- Monitor database query performance
- Review OpenTelemetry traces for bottlenecks

### Development Tips

- Use `DEBUG=true` in your backend `.env` for detailed error messages during development
- Run `npm run build` in the frontend to check for build errors before deployment
- Check the API documentation at `/docs` endpoint when backend is running
- Use the health check endpoint (`/health`) to monitor system status

## Next Steps

1. Review the [Implementation Plan](plan.md) for detailed architecture
2. Check the [Data Model](data-model.md) for database structure
3. Explore the [API Contracts](contracts/) for integration details
4. Review the [Research](research.md) document for technical decisions