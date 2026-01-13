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

## Architecture

- **Backend**: FastAPI application (Python 3.11)
- **AI Engine**: OpenAI API (text-embedding-3-small, gpt-3.5-turbo)
- **Vector Database**: Qdrant Cloud
- **Database**: Neon PostgreSQL (serverless)
- **Deployment**: Hugging Face Spaces (Docker SDK)

## Quick Start

### Hugging Face Spaces Deployment

**Space:** `https://huggingface.co/spaces/sarimarain/ai-native-book`

1. **Clone the Space repository:**

   ```bash
   # When prompted for password, use an access token with write permissions
   # Generate one from: https://huggingface.co/settings/tokens
   git clone https://huggingface.co/spaces/sarimarain/ai-native-book
   cd ai-native-book
   ```

2. **Configure Secrets** in Settings → Variables and secrets:

| Variable               | Required | Description                                        |
| ---------------------- | -------- | -------------------------------------------------- |
| `OPENAI_API_KEY`       | Yes      | OpenAI API key for embeddings and chat             |
| `QDRANT_URL`           | Yes      | Qdrant Cloud cluster URL                           |
| `QDRANT_API_KEY`       | Maybe    | Qdrant API key (if using cloud)                    |
| `NEON_API_KEY`         | Yes      | PostgreSQL connection string (Neon)                |
| `BETTER_AUTH_SECRET`   | Yes      | JWT secret for authentication                      |
| `BETTER_AUTH_URL`      | Yes      | Backend URL for auth callbacks (your HF Space URL) |
| `CORS_ALLOWED_ORIGINS` | Yes      | Comma-separated list of allowed frontend origins   |

3. **Copy backend files to the Space:**

   ```bash
   # Copy all backend files to the Space directory
   cp -r /path/to/backend/* .

   # Or add the Space as a remote and push
   git remote add hf https://huggingface.co/spaces/sarimarain/ai-native-book
   git push hf main
   ```

4. **Monitor build** in the Logs tab on Hugging Face
5. **Test deployment**: Visit `https://sarimarain-ai-native-book.hf.space` and check `/health` endpoint

### Local Development

1. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

2. Create `.env` file with required variables (see Hugging Face Secrets above)

3. Run the ingestion process:

   ```bash
   python ingest.py
   ```

4. Start the server:
   ```bash
   uvicorn main:app --host 0.0.0.0 --port 8080
   ```

## Environment Variables

### Required Variables

| Variable               | Description                  | Example                                |
| ---------------------- | ---------------------------- | -------------------------------------- |
| `OPENAI_API_KEY`       | OpenAI API key               | `sk-proj-...`                          |
| `QDRANT_URL`           | Qdrant Cloud cluster URL     | `https://...qdrant.io`                 |
| `NEON_API_KEY`         | PostgreSQL connection string | `postgresql://user:pass@...`           |
| `BETTER_AUTH_SECRET`   | JWT secret for auth          | (generate with `openssl rand -hex 32`) |
| `BETTER_AUTH_URL`      | Backend URL for callbacks    | `https://your-space.hf.space`          |
| `CORS_ALLOWED_ORIGINS` | Allowed frontend origins     | `https://your-frontend.com,...`        |

### Optional Variables

| Variable                      | Default         | Description                                      |
| ----------------------------- | --------------- | ------------------------------------------------ |
| `QDRANT_API_KEY`              | `None`          | Qdrant API key (optional for local instances)    |
| `QDRANT_COLLECTION_NAME`      | `claude-cli-db` | Qdrant collection name                           |
| `PORT`                        | `7860`          | Web server port (HF Spaces Docker SDK uses 7860) |
| `OTEL_EXPORTER_OTLP_ENDPOINT` | -               | OpenTelemetry exporter endpoint                  |
| `OTEL_CONSOLE_EXPORT`         | -               | Enable console export for tracing                |

## Hugging Face Space URL Pattern

Your Hugging Face Space URL follows this pattern:

```
https://<username>-<space-name>.hf.space
```

Add this URL to `CORS_ALLOWED_ORIGINS` to allow frontend requests.

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
- `POST /ingest` - Parse docs and store embeddings

## Deployment Instructions

### Deploying to ai-native-book Space

**Space URL:** `https://huggingface.co/spaces/sarimarain/ai-native-book`

**Method 1: Using Git**

```bash
# 1. Clone the Space
git clone https://huggingface.co/spaces/sarimarain/ai-native-book
cd ai-native-book

# 2. Copy backend files to the Space
# From your project root:
cp -r backend/* /path/to/ai-native-book/

# 3. Commit and push
git add .
git commit -m "Deploy backend to HF Spaces"
git push
```

**Method 2: Using Hugging Face CLI**

```bash
# 1. Install HF CLI (PowerShell - Windows)
powershell -ExecutionPolicy ByPass -c "irm https://hf.co/cli/install.ps1 | iex"

# 2. Login with your access token (with write permissions)
huggingface-cli login

# 3. Download and deploy
hf download sarimarain/ai-native-book --repo-type=space
# Copy files, then:
cd ai-native-book
git add .
git commit -m "Deploy backend"
git push
```

**Method 3: Using Web UI**

1. Go to `https://huggingface.co/spaces/sarimarain/ai-native-book`
2. Click "Files" tab
3. Click "Upload files"
4. Upload these files:
   - `Dockerfile`
   - `README.md`
   - `requirements.txt`
   - `config.py`
   - `main.py`
   - `src/` (entire directory)
5. Click "Commit changes to main"

## Troubleshooting

### Build Fails

- Check the Logs tab for specific error messages
- Verify `requirements.txt` has all dependencies
- Check for syntax errors in Python files

### Service Won't Start

- Check Logs for startup errors
- Verify all required Secrets are configured
- Check `CORS_ALLOWED_ORIGINS` format (comma-separated, no spaces)

### CORS Errors

- Verify frontend URL is in `CORS_ALLOWED_ORIGINS`
- Ensure URL includes protocol (`https://`)
- Check for typos in origin URLs

### External Service Connection Failures

- Verify API keys are correct
- Check if services allow traffic from Hugging Face IPs
- Ensure connection strings use correct format:
  - Neon: `postgresql://...` (not `postgres://`)
  - Qdrant: Full URL with port

### PORT Variable Issues

- HF Spaces Docker SDK uses port **7860** by default
- Dockerfile CMD: `uvicorn main:app --host 0.0.0.0 --port ${PORT:-7860}`
- The Space should automatically detect the correct port
- If you see "Connection refused", verify the CMD is using `${PORT}` variable

## Security

- All API keys stored as Hugging Face Secrets (never in code)
- CORS restricts access to authorized frontend domains
- Error messages don't expose sensitive configuration details
- JWT-based authentication via Better Auth

## Performance

- API endpoints respond within 5 seconds for 95% of requests
- Health check responds within 1 second
- Service startup completes within 30 seconds
- Handles 50+ concurrent requests

## License

MIT License - see LICENSE file for details

## Credits

Part of the AI-Native Textbook on Physical AI & Humanoid Robotics for Panaversity Hackathon I.
