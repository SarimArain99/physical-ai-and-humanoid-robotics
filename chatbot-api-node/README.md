# Physical AI Book — Backend (Express)

The backend for the Physical AI & Humanoid Robotics book: email/password
login (better-auth), AI chat with RAG over the book's chapters, Urdu
translation, and level adjustment (beginner / intermediate / pro).

- **Live:** https://physical-ai-backend-mu.vercel.app
- **Vercel project:** `physical-ai-backend` (same Vercel account as the book
  frontend). The frontend calls this URL — never localhost, never the old
  Hugging Face backend.
- **Deployed as:** one Vercel serverless function (`api/index.js` exports the
  Express app; it only calls `app.listen()` when `VERCEL` is not set).

## Where this code came from

Copied from `SarimArain99/Physical-AI-Humanoid-Robotics` (`chatbot-api-node/`,
the backend that already ran on Vercel since August 2026). That repo is
untouched; this copy is the book's own backend from here on, so the whole
system lives in one repo and one Vercel account.

## Changes made in this copy

1. **RAG context is now this book's chapters.** The original loaded
   `../textbook/docs` (the other repo's docs, a different structure). This
   copy resolves the docs relative to its own file —
   `../frontend/docs` — so answers are grounded in the 13 chapters of this
   book. Set `TEXTBOOK_DOCS_PATH` to point it elsewhere.
2. **`/api/diagnostics` removed.** The old endpoint returned masked
   prefixes of live secrets (`DATABASE_URL`, `OPENAI_API_KEY`,
   `BETTER_AUTH_SECRET`). A debug endpoint has no place in production.
   Use `/api/health` for status checks instead.
3. Nothing else changed: same routes, same auth, same rate limits, same
   streaming (SSE) responses.

## Routes

- `POST /api/auth/*` — better-auth (sign-up, sign-in, session, sign-out)
- `POST /api/chat` — RAG chat, SSE stream (`{ message, session_id }`)
- `POST /api/chat/selected` — chat about highlighted text, SSE
- `POST /api/translate` — translate text (default Urdu), SSE
- `POST /api/personalize` — rewrite text for a level
  (`beginner` / `intermediate`), SSE
- `POST /api/quiz`, `POST /api/flashcards` — generated study material
- `POST /api/progress`, `GET /api/progress/:userId` — reading progress
- `GET /api/cache/:key`, `POST /api/cache` — translation cache
- `GET /api/health` — `{ "status": "ok" }`

Chat history, progress and cache live in Neon Postgres (same database the
auth sessions use). AI calls use `gpt-4.1-nano`, rate-limited to 60 AI
requests per IP per 15 minutes.

## Environment variables (already set on the Vercel project)

- `DATABASE_URL` — Neon Postgres connection string
- `OPENAI_API_KEY` — OpenAI API key
- `BETTER_AUTH_SECRET` — better-auth secret
- `BETTER_AUTH_URL` — the backend's own production URL
- `CORS_ORIGINS` — comma-separated allowed frontend origins
  (any `*.vercel.app` origin is also allowed by the code)
- `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME` — set on the
  project but unused by this code (leftover from an earlier design)
- Optional: `TEXTBOOK_DOCS_PATH`, `SMTP_HOST`, `SMTP_PORT`,
  `SMTP_SECURE`, `SMTP_USER`, `SMTP_PASS`, `SMTP_FROM` (password-reset mail;
  without SMTP set, reset links are only logged, not emailed)

Never put real values in this repo. Values live only in the Vercel
project settings.

## Running locally

```bash
cd chatbot-api-node
npm install
# create a .env with DATABASE_URL, OPENAI_API_KEY, BETTER_AUTH_SECRET,
# BETTER_AUTH_URL=http://localhost:8000 and CORS_ORIGINS for the frontend
npm start   # listens on http://localhost:8000
```

## Deploying

The backend is deployed to the existing `physical-ai-backend` Vercel
project as a **file deployment** (the project is not git-linked, so pushes
do not auto-deploy it). Deploy with the Vercel `create_deployment` call,
target `production`, with these files:

- `api/index.js` ← `chatbot-api-node/index.js`
- `api/auth.js` ← `chatbot-api-node/auth.js`
- `package.json`, `package-lock.json`, `.npmrc` at the deployment root
  ← `chatbot-api-node/package.json`, `package-lock.json`, `.npmrc`
- `frontend/docs/*.md` ← `frontend/docs/*.md` (the RAG context)

The root `package.json` (`"type": "module"`) is what makes `api/*.js`
load as ESM and lets Vercel install dependencies. The legacy
`vercel.json` in this folder is kept for reference only (it was the old
`@vercel/node` builds config); it is **not** part of the file deployment
— the `api/` layout needs no `vercel.json`.

After deploying, check `GET /api/health` on the production URL and confirm
the deployment state is READY before calling it live. The frontend needs
no change: it already calls the production URL.

## The FastAPI backend

The repo also contains `backend/` (FastAPI). It is **not** deployed and
not used by the frontend — it was the original Hugging Face backend. It is
kept for reference only. Do not point the frontend at it.
