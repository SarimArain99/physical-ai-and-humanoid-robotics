# Quickstart: Hugging Face Backend Deployment

**Feature**: 001-huggingface
**Phase**: 1 - Design & Contracts
**Date**: 2026-01-13

## Overview

This guide provides step-by-step instructions for deploying the Physical AI & Humanoid Robotics backend to Hugging Face Spaces using Docker SDK.

---

## Prerequisites

Before deploying, ensure you have:

1. **Hugging Face Account**
   - Sign up at https://huggingface.co
   - Verify your email address

2. **External Service Accounts**
   - [Neon PostgreSQL](https://neon.tech/) - Free tier sufficient
   - [Qdrant Cloud](https://qdrant.tech/) - Free tier sufficient
   - [OpenAI API](https://platform.openai.com/) - API key with credits

3. **Git Repository Access**
   - Access to the backend code repository
   - Git installed locally

4. **Frontend URL** (for CORS configuration)
   - Your deployed frontend URL (e.g., Vercel URL)

---

## Step 1: Create a New Hugging Face Space

1. Go to https://huggingface.co/spaces
2. Click **"Create new Space"**
3. Configure the Space:
   - **Owner**: Your username or organization
   - **Space name**: `physical-ai-backend` (or your preferred name)
   - **License**: MIT or Apache-2.0
   - **SDK**: Docker
   - **Hardware**: CPU basic (free tier) or CPU upgrade if needed
4. Click **"Create Space"**

Your Space URL will be: `https://<username>-physical-ai-backend.hf.space`

---

## Step 2: Configure Secrets

In your Space, go to **Settings** → **Variables and secrets** and add the following:

| Variable | Value | Required |
|----------|-------|----------|
| `OPENAI_API_KEY` | Your OpenAI API key (`sk-...`) | Yes |
| `QDRANT_URL` | Your Qdrant Cloud URL (e.g., `https://...qdrant.io`) | Yes |
| `QDRANT_API_KEY` | Your Qdrant API key | Maybe (if using cloud) |
| `NEON_API_KEY` | PostgreSQL connection string (`postgresql://...`) | Yes |
| `BETTER_AUTH_SECRET` | Random string (generate: `openssl rand -hex 32`) | Yes |
| `BETTER_AUTH_URL` | Your HF Space URL (e.g., `https://username-backend.hf.space`) | Yes |
| `CORS_ALLOWED_ORIGINS` | Frontend URL(s) comma-separated | Yes |

### Example CORS_ALLOWED_ORIGINS:
```
https://physical-ai-and-humanoid-robotics-omega.vercel.app,https://your-username-physical-ai-backend.hf.space
```

### Generate BETTER_AUTH_SECRET:
```bash
openssl rand -hex 32
```

---

## Step 3: Deploy via Git Push

### Option A: Using Hugging Face CLI (Recommended)

```bash
# Install HF CLI
pip install huggingface_hub

# Login
huggingface-cli login

# Navigate to backend directory
cd backend/

# Initialize git if needed
git init

# Add remote (replace USERNAME and SPACE_NAME)
git remote add hf https://huggingface.co/spaces/USERNAME/physical-ai-backend

# Add and commit files
git add .
git commit -m "Initial Hugging Face deployment"

# Push to Hugging Face
git push hf main
```

### Option B: Using Web UI

1. Go to your Space on Hugging Face
2. Click **"Files"** tab
3. Click **"Add file"** → **"Upload files"**
4. Upload the entire `backend/` folder contents
5. Click **"Commit changes to main"**

---

## Step 4: Monitor Build Progress

1. Go to your Space on Hugging Face
2. View the **"Logs"** tab
3. Wait for the build to complete (typically 5-15 minutes)
4. Look for these success messages:
   - `✅ Connected to Qdrant collection`
   - `✅ Database tables created successfully`
   - `Application startup complete`

---

## Step 5: Verify Deployment

### Health Check

```bash
curl https://USERNAME-physical-ai-backend.hf.space/health
```

Expected response:
```json
{"status": "healthy"}
```

### Circuit Status Check

```bash
curl https://USERNAME-physical-ai-backend.hf.space/health/circuits
```

Expected response:
```json
{
  "openai": {"state": "closed", "failure_count": 0},
  "qdrant": {"state": "closed", "failure_count": 0}
}
```

---

## Step 6: Update Frontend Configuration

Update your frontend's backend URL to point to the new Hugging Face Space:

```javascript
// Frontend configuration
const BACKEND_URL = 'https://USERNAME-physical-ai-backend.hf.space';
```

Add this URL to `CORS_ALLOWED_ORIGINS` in your Hugging Face Space secrets if not already added.

---

## Troubleshooting

### Build Fails

**Issue**: Build timeout or fails

**Solutions**:
- Check the Logs tab for specific error messages
- Verify `requirements.txt` has all dependencies
- Check for syntax errors in Python files
- Ensure Dockerfile is in the backend root

### Service Won't Start

**Issue**: Space builds but service returns errors

**Solutions**:
1. Check Logs for startup errors
2. Verify all required secrets are configured
3. Check `CORS_ALLOWED_ORIGINS` format (comma-separated, no spaces)
4. Verify external service URLs are correct

### CORS Errors

**Issue**: Frontend gets CORS errors when calling API

**Solutions**:
1. Verify frontend URL is in `CORS_ALLOWED_ORIGINS`
2. Ensure URL includes protocol (`https://`)
3. Check for typos in origin URLs
4. Verify no trailing slashes in origins

### External Service Connection Failures

**Issue**: Cannot connect to Neon, Qdrant, or OpenAI

**Solutions**:
1. Verify API keys are correct
2. Check if services allow traffic from Hugging Face IPs
3. Ensure connection strings use correct format:
   - Neon: `postgresql://...` (not `postgres://`)
   - Qdrant: Full URL with port

### PORT Variable Issues

**Issue**: Service can't bind to port

**Solutions**:
- Dockerfile already handles `PORT` environment variable
- Should auto-detect HF's port
- Check if another process is using the port

---

## Environment Variables Reference

| Variable | Description | Example |
|----------|-------------|---------|
| `OPENAI_API_KEY` | OpenAI API key | `sk-proj-...` |
| `QDRANT_URL` | Qdrant cluster URL | `https://...qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | `...` (optional) |
| `QDRANT_COLLECTION_NAME` | Collection name | `claude-cli-db` |
| `NEON_API_KEY` | PostgreSQL connection | `postgresql://user:pass@...` |
| `BETTER_AUTH_SECRET` | JWT secret | (random hex string) |
| `BETTER_AUTH_URL` | Backend URL | `https://...hf.space` |
| `CORS_ALLOWED_ORIGINS` | Allowed origins | `https://frontend.com,...` |
| `PORT` | Server port | (auto-set by HF) |

---

## Post-Deployment Checklist

- [ ] Health endpoint returns `{"status": "healthy"}`
- [ ] Circuit endpoints show `closed` state for both services
- [ ] Frontend can make API requests without CORS errors
- [ ] Chat endpoint responds with context-aware answers
- [ ] Authentication works (JWT tokens valid)
- [ ] Logs show successful external service connections

---

## Next Steps

After successful deployment:

1. Test all API endpoints from your frontend
2. Monitor logs for any errors
3. Set up monitoring for production usage
4. Consider upgrading hardware tier if needed for higher traffic

---

## Additional Resources

- [Hugging Face Spaces Documentation](https://huggingface.co/docs/hub/spaces)
- [Docker SDK Guide](https://huggingface.co/docs/hub/spaces-sdks-docker)
- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
