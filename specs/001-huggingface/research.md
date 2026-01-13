# Research: Hugging Face Spaces Deployment

**Feature**: 001-huggingface
**Phase**: 0 - Research & Technology Decisions
**Date**: 2026-01-13

## Overview

This document consolidates research findings for deploying the FastAPI backend to Hugging Face Spaces using Docker SDK. Topics include HF Spaces constraints, configuration management patterns, CORS setup, and Docker optimization.

---

## Topic 1: Hugging Face Spaces Docker SDK

### Decision: Use Docker SDK with Python 3.11 slim

**Rationale**:
- Hugging Face Spaces Docker SDK provides full containerization control
- Python 3.11 slim base image balances compatibility and size
- Existing Dockerfile already uses correct PORT pattern: `${PORT:-8080}`
- Docker SDK allows pre-built images and caching for faster deployments

**HF Spaces Constraints**:

| Constraint | Limit | Mitigation |
|------------|-------|------------|
| Build time | ~30 minutes | Use cached layers, optimize dependencies |
| Storage | ~20GB for Docker image | Multi-stage build, .dockerignore |
| CPU | 2-8 vCPUs depending on tier | Stateless design, external services |
| Memory | 16GB-48GB depending on tier | Minimal in-memory state |
| Port | Must use `PORT` env variable | Already handled in Dockerfile |

**Best Practices**:
1. Use `.dockerignore` to exclude unnecessary files
2. Copy `requirements.txt` before source code for better layer caching
3. Use `--no-cache-dir` for pip to reduce image size
4. Set `WORKDIR` explicitly
5. Use `CMD` with shell form for variable expansion

### PORT Variable Handling

**Current Implementation (Already Correct)**:
```dockerfile
CMD ["sh", "-c", "uvicorn main:app --host 0.0.0.0 --port ${PORT:-8080}"]
```

**HF Behavior**:
- HF sets `PORT` environment variable automatically
- Default is 7860 for Spaces (but varies)
- Our fallback to 8080 ensures local development still works

**Verdict**: No changes needed to Dockerfile for PORT handling.

---

## Topic 2: FastAPI Configuration with Pydantic Settings

### Decision: Make env_file optional in Pydantic SettingsConfigDict

**Rationale**:
- Hugging Face Spaces uses Secrets (environment variables), not `.env` files
- Pydantic Settings defaults to requiring `env_file` to exist
- Making `env_file` optional allows deployment without `.env`

**Current Code Issue** (`backend/config.py`):
```python
ENV_FILE_PATH = os.path.join(BASE_DIR, ".env")
model_config = SettingsConfigDict(
    env_file=ENV_FILE_PATH,  # <-- Fails if .env doesn't exist
    env_file_encoding="utf-8",
    extra="ignore"
)
```

**Required Change**:
```python
# Make env_file optional
env_file = ENV_FILE_PATH if os.path.exists(ENV_FILE_PATH) else None
model_config = SettingsConfigDict(
    env_file=env_file,  # Can be None
    env_file_encoding="utf-8",
    extra="ignore"
)
```

**Alternative Considered**: Create empty `.env` file during Docker build
- **Rejected**: Adds complexity, unnecessary I/O
- **Better**: Use None when file doesn't exist

### Missing Configuration Error Handling

**Current Behavior**: Pydantic raises `ValidationError` for missing required fields

**Enhancement Required**: Clear error messages for missing secrets
- Wrap Settings initialization
- Check for missing required values before startup
- Log which specific environment variables are missing

---

## Topic 3: CORS Configuration for Hugging Face Spaces

### Decision: Use environment variable for configurable origins

**Rationale**:
- HF Space URLs follow pattern: `https://username-spacename.hf.space`
- Space URL may not be known until after deployment
- Environment variable allows runtime configuration
- Frontend URL (Vercel) remains constant

**Current Issue** (`backend/main.py`):
```python
default_origins = [
    "http://localhost:3000",
    "http://localhost:3001",
    "https://physical-ai-and-humanoid-robotics-omega.vercel.app",
    "https://physical-ai-and-humanoid-robotics-production.up.railway.app"  # <-- Railway URL
]
```

**Required Changes**:
1. Remove Railway URL from default_origins
2. Add placeholder comment for HF Space URL
3. Ensure `CORS_ALLOWED_ORIGINS` env variable is parsed correctly
4. Document HF Space URL pattern in README

**HF Space URL Pattern**:
```
https://<username>-<space-name>.hf.space
```

**Example Configuration**:
```bash
# In HF Secrets
CORS_ALLOWED_ORIGINS=https://physical-ai-frontend.vercel.app,https://username-backend.hf.space
```

### Preflight Request Handling

**Current Implementation** (Already Correct):
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,  # <-- Required for auth
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Verdict**: No changes needed to CORS middleware configuration.

---

## Topic 4: Dockerfile Optimization

### Decision: Verify and optimize existing Dockerfile

**Current Dockerfile Analysis**:
```dockerfile
FROM python:3.11-slim          # ✅ Good: slim base image
WORKDIR /app                    # ✅ Good: explicit workdir
COPY requirements.txt .         # ✅ Good: separate layer for caching
RUN pip install --no-cache-dir -r requirements.txt  # ✅ Good: no cache
COPY . .                        # ✅ Good: copy after dependencies
CMD ["sh", "-c", "uvicorn main:app --host 0.0.0.0 --port ${PORT:-8080}"]
```

**Optimizations Already in Place**:
- Layer caching (requirements.txt before source)
- `--no-cache-dir` for pip
- Slim base image

**Potential Improvements** (Optional):
1. Multi-stage build (not needed for this deployment)
2. Combine RUN commands (minor optimization)
3. Add `.dockerignore` entries for development files

**.dockerignore Review** (Already Well-Configured):
- Excludes `__pycache__`, `.pyc`, virtual environments
- Excludes `.env`, `.env.local` (correct for HF)
- Excludes `.git`, IDE folders
- Excludes test artifacts

**Verdict**: Dockerfile is production-ready for HF Spaces. No major changes needed.

---

## Topic 5: External Service Connectivity

### Decision: Verify external services accessible from HF infrastructure

**Services**:

| Service | Protocol | Port | Connectivity Required |
|---------|----------|------|----------------------|
| Neon PostgreSQL | PostgreSQL | 5432 | ✅ Public cloud, accessible |
| Qdrant Cloud | HTTP/HTTPS | 443 | ✅ Public cloud, accessible |
| OpenAI API | HTTPS | 443 | ✅ Public API, accessible |

**Verification Strategy**:
1. Check if service URLs allow HF Spaces IPs
2. Test connectivity during first deployment
3. Add connection logging to lifespan startup

**Current Implementation** (Already Has Logging):
```python
# In lifespan function
try:
    app.state.qdrant_client.get_collection(settings.qdrant_collection_name)
    logger.info(f"✅ Connected to Qdrant collection: {settings.qdrant_collection_name}")
except Exception as e:
    logger.error(f"⚠️ Could not connect to Qdrant collection: {e}")
```

**Verdict**: Existing connectivity checks sufficient. No changes needed.

---

## Summary of Decisions

| Topic | Decision | Change Required |
|-------|----------|-----------------|
| Docker SDK | Use existing Dockerfile | No (verify only) |
| PORT variable | Already handled | No |
| env_file option | Make env_file optional | Yes (config.py) |
| Missing config errors | Add clear error messages | Yes (config.py) |
| CORS origins | Remove Railway URL, add HF pattern | Yes (main.py) |
| External services | Verify connectivity | No (existing logs sufficient) |
| Dockerfile optimization | Already optimized | No |
| .dockerignore | Already configured | No |

---

## Technology Choices Verified

| Technology | Version | Source |
|------------|---------|--------|
| Python | 3.11-slim | Hugging Face recommendation |
| FastAPI | 0.104.1 | Existing dependency |
| Pydantic | 2.x | Existing dependency |
| Docker SDK | Docker | Hugging Face platform |

**No NEEDS CLARIFICATION items remain** - all technical decisions are complete.
