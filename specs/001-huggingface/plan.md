# Implementation Plan: Hugging Face Backend Deployment

**Branch**: `001-huggingface` | **Date**: 2026-01-13 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-huggingface/spec.md`

## Summary

Migrate the Physical AI & Humanoid Robotics backend from Railway to Hugging Face Spaces using Docker SDK. The backend is a FastAPI application providing RAG-powered chatbot services with PostgreSQL (Neon), vector database (Qdrant Cloud), and OpenAI API integration. The migration focuses on configuration management for Hugging Face's environment-based secrets, CORS updates, and Docker optimization.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI 0.104.1, Uvicorn 0.24.0, Pydantic 2.x, SQLAlchemy 2.0, OpenAI 1.3.6, Qdrant-client 1.8.0
**Storage**: Neon PostgreSQL (external cloud database), Qdrant Cloud (external vector database)
**Testing**: pytest (existing), health endpoints for deployment verification
**Target Platform**: Hugging Face Spaces (Docker SDK) - Linux container
**Project Type**: Web application (backend API)
**Performance Goals**:
- API endpoint response: <5 seconds for 95% of requests
- Health check: <1 second
- Service startup: <30 seconds
- Concurrent requests: 50+ without degradation
**Constraints**:
- Docker build time: <20 minutes (HF limit is ~30 minutes)
- Docker image size: Optimize for HF storage limits
- No `.env` file dependency (use HF Secrets)
- PORT environment variable must be respected
**Scale/Scope**:
- Single backend service
- External services: Neon PostgreSQL, Qdrant Cloud, OpenAI API
- Zero downtime migration (backend stateless, data in external services)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Compliance | Notes |
|-----------|-------------|-------|
| **AI-Native Pedagogy** | ✅ PASS | Backend supports RAG chatbot for interactive learning |
| **Technical Accuracy** | ✅ PASS | No changes to technical content, only deployment |
| **Accessibility & Inclusivity** | ✅ PASS | Maintains existing translation/personalization features |
| **Open & Reproducible** | ✅ PASS | Deployment to open HF Spaces, Docker configuration documented |
| **Interactive Learning** | ✅ PASS | RAG chatbot functionality preserved |
| **Required Tools** | ✅ PASS | Uses FastAPI, Neon, Qdrant as specified in constitution |
| **Technical Standards** | ✅ PASS | Python 3.11, FastAPI backend maintained |

**Gate Status**: ✅ PASSED - No constitution violations. Migration is purely deployment-focused with no changes to core functionality or content.

## Project Structure

### Documentation (this feature)

```text
specs/001-huggingface/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0: Hugging Face deployment research
├── data-model.md        # Phase 1: Configuration data model
├── quickstart.md        # Phase 1: Deployment quickstart guide
├── contracts/           # Phase 1: API contracts (existing API)
│   └── backend-api.yaml # OpenAPI specification for health/config endpoints
└── tasks.md             # Phase 2: Implementation tasks (NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── config.py            # MODIFY: Environment variable handling
├── main.py              # MODIFY: CORS origins update
├── Dockerfile           # VERIFY/UPDATE: HF Docker compatibility
├── .dockerignore        # VERIFY: Build optimizations
├── requirements.txt     # VERIFY: Dependencies
├── README.md            # CREATE: HF deployment documentation
├── nixpacks.toml        # REMOVE: Railway-specific (not needed for HF)
├── src/
│   ├── database.py      # Existing: Neon PostgreSQL integration
│   ├── auth_better.py   # Existing: Better Auth integration
│   └── services/
│       └── chat_service.py  # Existing: Chat business logic
└── tests/
    └── contract/        # Existing: API tests
```

**Structure Decision**: Existing backend structure preserved. Only configuration and deployment files modified. No changes to core application logic, database schema, or API endpoints.

## Complexity Tracking

> **No violations to justify** - This is a deployment migration with no architecture changes.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

---

## Phase 0: Research & Technology Decisions

### Research Topics

1. **Hugging Face Spaces Docker SDK**
   - Docker build constraints and best practices
   - Environment variable/Secrets management
   - PORT variable handling
   - Resource limits (CPU, memory, storage)

2. **FastAPI Configuration Patterns**
   - Pydantic Settings with optional env_file
   - Missing configuration error handling
   - Multiple deployment environment support

3. **CORS for Hugging Face Spaces**
   - Space URL patterns
   - Dynamic origin configuration
   - Preflight request handling

### Research Artifacts

**Output**: `specs/001-huggingface/research.md`

Will document:
- HF Spaces Docker SDK constraints and best practices
- Configuration management patterns for environment-only deployment
- CORS configuration for dynamic Space URLs
- Dockerfile optimization strategies

---

## Phase 1: Design & Contracts

### Data Model

**Output**: `specs/001-huggingface/data-model.md`

**Configuration Variables Entity**:
```yaml
ConfigurationVariable:
  attributes:
    - name: OPENAI_API_KEY
      required: true
      description: OpenAI API key for embeddings and chat
    - name: QDRANT_URL
      required: true
      description: Qdrant Cloud cluster URL
    - name: QDRANT_API_KEY
      required: false
      description: Qdrant API key (optional for local instances)
    - name: NEON_API_KEY
      required: true
      description: PostgreSQL connection string (Neon)
    - name: BETTER_AUTH_SECRET
      required: true
      description: JWT secret for authentication
    - name: BETTER_AUTH_URL
      required: true
      description: Backend URL for auth callbacks
    - name: CORS_ALLOWED_ORIGINS
      required: true
      description: Comma-separated list of allowed frontend origins
    - name: PORT
      required: false
      default: "8080"
      description: Web server port (set by HF automatically)
```

### API Contracts

**Output**: `specs/001-huggingface/contracts/backend-api.yaml`

Existing endpoints to verify (no changes to API contract):
- `GET /` - Root status endpoint
- `GET /health` - Health check
- `GET /health/circuits` - Circuit breaker status

### Quickstart Guide

**Output**: `specs/001-huggingface/quickstart.md`

Will contain:
1. Prerequisites (HF account, external service accounts)
2. Creating a new Hugging Face Space
3. Configuring Secrets
4. Deploying via Git push
5. Verifying deployment
6. Troubleshooting common issues

### Agent Context Update

**Output**: `.specify/memory/active-technologies.md`

Will add/update:
- Hugging Face Spaces (Docker SDK)
- Deployment configuration patterns

---

## Phase 2: Implementation (Out of Scope for /sp.plan)

Implementation tasks will be generated by `/sp.tasks` command.

**Key Implementation Areas** (to be broken into tasks):

1. **Configuration Management** (FR-001 to FR-004)
   - Modify `config.py` for optional env_file
   - Add clear error messages for missing required values

2. **Docker Optimization** (FR-005 to FR-008)
   - Verify/optimize Dockerfile for HF Spaces
   - Update .dockerignore if needed

3. **CORS Updates** (FR-009 to FR-012)
   - Update `main.py` default origins
   - Remove Railway-specific URLs
   - Add HF Space URL pattern support

4. **Documentation** (FR-022 to FR-025)
   - Create HF-specific README.md
   - Document all environment variables
   - Add deployment and troubleshooting guides

5. **Cleanup**
   - Remove `nixpacks.toml` (Railway-specific)

---

## Dependencies & Risks

### External Dependencies

| Service | Risk | Mitigation |
|---------|------|------------|
| Hugging Face Spaces | Platform downtime | Document alternative deployment |
| Neon PostgreSQL | Connectivity from HF | Verify network access, test early |
| Qdrant Cloud | Connectivity from HF | Verify network access, test early |
| OpenAI API | Connectivity from HF | No change, already external |

### Implementation Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| HF build timeout | High | Optimize Dockerfile, test builds locally |
| CORS misconfiguration | Medium | Document testing procedure, provide env var override |
| Missing secrets at runtime | High | Clear error messages, documentation |
| PORT variable issues | Low | Already handled correctly in Dockerfile |

---

## Success Criteria Validation

Each success criterion from spec.md mapped to verification:

| SC | Metric | Verification Method |
|----|--------|---------------------|
| SC-001 | <5 min accessibility | Deploy and test `/health` endpoint |
| SC-002 | External services connected | Check logs for successful connections |
| SC-003 | <2s health response | `curl -w @- format.txt` timing test |
| SC-004 | CORS works | Cross-origin request from frontend |
| SC-005 | 50 concurrent requests | Load test with `ab` or similar |
| SC-006 | <20 min build | Monitor HF build logs |
| SC-007 | All env vars documented | Review README.md |
| SC-008 | Circuit breaker recovery | Test external service failure simulation |
