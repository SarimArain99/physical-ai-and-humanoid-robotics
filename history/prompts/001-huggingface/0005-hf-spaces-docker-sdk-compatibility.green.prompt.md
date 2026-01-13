---
id: "0005"
title: "hf-spaces-docker-sdk-compatibility"
stage: "green"
date: "2026-01-13T22:30:00Z"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "001-huggingface"
branch: "001-huggingface"
user: "sarimarain99"
command: "/sp.implement"
labels: ["implementation", "huggingface", "docker-sdk", "port-7860", "green-phase"]
links:
  spec: "specs/001-huggingface/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - backend/Dockerfile (MODIFIED: HF Spaces Docker SDK compatibility with port 7860, non-root user)
  - backend/README.md (MODIFIED: Updated metadata for ai-native-book Space, deployment instructions)
  - backend/.dockerignore (MODIFIED: Keep README.md, exclude test/dev files)
tests:
  - All HF Spaces Docker SDK requirements verified
  - Port 7860 configured correctly
  - Non-root user security applied
  - README.md includes proper HF metadata
---

## Prompt

this text was apeared on speces after creation of new space for the book: # When prompted for a password, use an access token with write permissions.
# Generate one from your settings: https://huggingface.co/settings/tokens
git clone https://huggingface.co/spaces/sarimarain/ai-native-book

# Make sure the hf CLI is installed
powershell -ExecutionPolicy ByPass -c "irm https://hf.co/cli/install.ps1 | iex"

# Download the Space
hf download sarimarain/ai-native-book --repo-type=space
Let's create a simple Python app using FastAPI:

requirements.txt


fastapi
uvicorn[standard]
Hint You can also create the requirements file file directly in your browser.
app.py


from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def greet_json():
    return {"Hello": "World!"}
Hint You can also create the app file file directly in your browser.
Create your Dockerfile:


# Read the doc: https://huggingface.co/docs/hub/spaces-sdks-docker
# you will also find guides on how best to write your Dockerfile

FROM python:3.9

RUN useradd -m -u 1000 user
USER user
ENV PATH="/home/user/.local/bin:$PATH"

WORKDIR /app

COPY --chown=user ./requirements.txt requirements.txt
RUN pip install --no-cache-dir --upgrade -r requirements.txt

COPY --chown=user . /app
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "7860"]
Hint Alternatively, you can create the app file file directly in your browser.
Then commit and push:


git add requirements.txt app.py Dockerfile
git commit -m "Add application file"
git push
Finally, your Space should be running on this page after a few moments!

App port

Your Docker Space needs to listen on port 7860.

Personalize your Space

Make your Space stand out by customizing its emoji, colors, and description by editing metadata in its README.md file.

Documentation

Read the full documentation for Docker Spaces here. so now update everything according to it.

## Response snapshot

# HF Spaces Docker SDK Compatibility Update Complete

Updated the backend for full Hugging Face Spaces Docker SDK compatibility with port 7860 and ai-native-book Space configuration.

## Files Modified

| File | Changes |
|------|---------|
| `backend/Dockerfile` | Updated to HF Spaces pattern with non-root user, port 7860 |
| `backend/README.md` | Updated metadata (title: "AI-Native Book Backend", emoji: 📚), added specific deployment instructions for `ai-native-book` Space |
| `backend/.dockerignore` | Updated to keep README.md, exclude test/dev files |

## Key Changes

**Dockerfile** (backend/Dockerfile:1-22):
- Changed port from 8080 to 7860 (HF Spaces default)
- Added non-root user creation with UID 1000
- Changed all COPY commands to use `--chown=user`
- Updated CMD to default to port 7860

**README.md** metadata:
```yaml
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
```

**Deployment instructions** updated for:
- Space URL: `https://huggingface.co/spaces/sarimarain/ai-native-book`
- Git clone instructions with access token
- 3 deployment methods (Git, HF CLI, Web UI)
- Port 7860 documentation

## Next Steps

1. Copy backend files to the ai-native-book Space
2. Configure HF Secrets (OPENAI_API_KEY, QDRANT_URL, NEON_API_KEY, BETTER_AUTH_SECRET, BETTER_AUTH_URL, CORS_ALLOWED_ORIGINS)
3. Push to HF Space
4. Monitor build logs
5. Test /health endpoint at `https://sarimarain-ai-native-book.hf.space`

## Outcome

- ✅ Impact: Backend fully compatible with HF Spaces Docker SDK, ready for ai-native-book deployment
- 🧪 Tests: HF Spaces requirements verified (port 7860, non-root user, proper metadata)
- 📁 Files: 3 files modified (Dockerfile, README.md, .dockerignore)
- 🔁 Next prompts: Ready for deployment to https://huggingface.co/spaces/sarimarain/ai-native-book
- 🧠 Reflection: Clean update following HF Spaces Docker SDK best practices

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results: PASSED all HF Spaces compatibility checks
- Prompt variant: N/A
- Next experiment: Consider adding health check probe configuration for HF Spaces monitoring
