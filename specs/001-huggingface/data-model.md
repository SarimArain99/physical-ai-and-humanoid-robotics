# Data Model: Hugging Face Backend Deployment

**Feature**: 001-huggingface
**Phase**: 1 - Design & Contracts
**Date**: 2026-01-13

## Overview

This document defines the configuration data model for the Hugging Face deployment. Since this is a deployment migration with no database schema changes, the data model focuses on environment configuration variables and deployment settings.

---

## Configuration Variables Entity

### Primary Entity: ConfigurationVariable

Represents all environment-based configuration settings required for the backend to operate on Hugging Face Spaces.

### Attributes

| Variable Name | Type | Required | Default | Description |
|----------------|------|----------|---------|-------------|
| `OPENAI_API_KEY` | string | Yes | - | OpenAI API key for embeddings and chat completions |
| `QDRANT_URL` | string | Yes | - | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | string | No | None | Qdrant API key (optional for local instances) |
| `QDRANT_COLLECTION_NAME` | string | No | `claude-cli-db` | Name of Qdrant collection for vector storage |
| `NEON_API_KEY` | string | Yes | - | PostgreSQL connection string (Neon) |
| `BETTER_AUTH_SECRET` | string | Yes | - | JWT secret for authentication signing |
| `BETTER_AUTH_URL` | string | No | `http://localhost:3000` | Backend URL for auth callbacks |
| `CORS_ALLOWED_ORIGINS` | string | No | (empty) | Comma-separated list of allowed frontend origins |
| `PORT` | integer | No | `8080` | Web server port (auto-set by HF Spaces) |
| `OTEL_EXPORTER_OTLP_ENDPOINT` | string | No | - | OpenTelemetry exporter endpoint (optional) |
| `OTEL_CONSOLE_EXPORT` | boolean | No | - | Enable console export for tracing (optional) |

### Validation Rules

1. **Required Variables**: Must be present at startup or service fails with clear error
2. **URL Validation**: URLs must be valid HTTPS endpoints where applicable
3. **CORS Origins**: Must be valid URLs separated by commas
4. **PORT**: Must be a valid integer between 1024 and 65535

### State Transitions

Configuration is loaded once at application startup:

```
Application Start → Load Configuration → Validate → [PASS] → Running
                                              |
                                              v
                                          [FAIL] → Error Message → Exit
```

---

## Hugging Face Space Entity

### Entity: HuggingFaceSpace

Represents the deployed application instance on Hugging Face infrastructure.

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `space_url` | string | Full URL of the deployed Space (e.g., `https://username-backend.hf.space`) |
| `sdk_type` | enum | Deployment method: `docker` |
| `space_name` | string | Name portion of the Space URL |
| `username` | string | Hugging Face username |
| `status` | enum | Build status: `building`, `running`, `stopped`, `error` |
| `secrets` | map | Key-value pairs of configured secrets |

### Relationships

- **Contains**: BackendApplication (1:1)
- **Exposes**: APIEndpoints (1:N)

---

## CORS Origin Entity

### Entity: CORSOrigin

Represents a frontend domain allowed to make API requests to the backend.

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `origin_url` | string | The complete origin URL (including protocol) |
| `environment` | enum | `production` or `development` |
| `enabled` | boolean | Whether this origin is currently allowed |

### Validation Rules

1. Origin must include protocol (http:// or https://)
2. Wildcards are NOT supported (security requirement)
3. Origins are case-sensitive

### Example Values

```
https://physical-ai-and-humanoid-robotics-omega.vercel.app (production)
http://localhost:3000 (development)
https://username-backend.hf.space (production - HF Space itself)
```

---

## Configuration Loading Flow

### Startup Sequence

```
1. Application Startup
   ↓
2. Check for .env file (optional)
   ↓
3. Load environment variables (from HF Secrets)
   ↓
4. Validate required variables present
   ├─ PASS → Continue
   └─ FAIL → Log error and exit
   ↓
5. Parse CORS_ALLOWED_ORIGINS
   ↓
6. Initialize external service clients
   ├─ OpenAI (with OPENAI_API_KEY)
   ├─ Qdrant (with QDRANT_URL, QDRANT_API_KEY)
   └─ Neon (with NEON_API_KEY)
   ↓
7. Start web server (on PORT)
```

### Error States

| Error | Condition | Resolution |
|-------|-----------|------------|
| Missing Required Secret | OPENAI_API_KEY not set | Configure in HF Secrets |
| Invalid URL | QDRANT_URL malformed | Fix URL format in Secrets |
| CORS Parse Error | Invalid origin format | Fix CORS_ALLOWED_ORIGINS format |
| Connection Failure | External service unreachable | Check service availability |

---

## No Database Changes

**Important**: This deployment migration does NOT modify any database schema. All existing tables and relationships remain unchanged:

- `users` table
- `user_profiles` table
- `chat_sessions` table
- `chat_messages` table

These entities are fully defined in the existing backend codebase (`backend/src/database.py`) and are not modified by this feature.

---

## Configuration File Impact

### Files Modified

| File | Change Type | Description |
|------|-------------|-------------|
| `backend/config.py` | MODIFY | Make env_file optional, add validation |
| `backend/main.py` | MODIFY | Update CORS default origins |

### Files Created

| File | Purpose |
|------|---------|
| `backend/README.md` | HF deployment documentation with YAML frontmatter |

### Files Removed

| File | Reason |
|------|--------|
| `backend/nixpacks.toml` | Railway-specific, not needed for HF |
