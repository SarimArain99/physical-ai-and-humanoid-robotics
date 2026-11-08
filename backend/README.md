# Backend — RETIRED (FastAPI) / LIVE (Express on Vercel)

## Live backend

The book's backend runs on **Vercel** (project `physical-ai-backend`,
live at `https://physical-ai-backend-mu.vercel.app`), from the
**`chatbot-api-node/`** directory in this repo (Express + better-auth,
OpenAI `gpt-4o-mini`, RAG over this book's own chapters).

The frontend (`frontend/src/config/api.js`) calls the Vercel URL only.
There is no Hugging Face dependency anywhere in the live stack.

## This directory (`backend/`, FastAPI) — retired

The FastAPI backend in this directory was the old implementation,
formerly deployed to Hugging Face Spaces. That deployment is dead
(503) and retired as of 2026-11-08. This code is kept for reference
only: it is not deployed, not imported by anything live, and its
Dockerfile is not used. Do not re-enable any Hugging Face deployment
from it.

## Live API (Express, `chatbot-api-node/`)

- `GET /api/health` — health check (also reports the OpenAI model)
- `POST /api/auth/sign-up`, `/api/auth/sign-in`, `/api/auth/sign-out` — auth
- `POST /api/chat` — RAG chat (Server-Sent Events stream)
- `POST /api/translate` — chapter translation (Urdu)
- `POST /api/personalize` — level rewrite (`basic` / `intermediate`;
  `pro` is the original text, no rewrite)

## Environment variables (Vercel project `physical-ai-backend`)

| Variable | Description |
| -------- | ----------- |
| `OPENAI_API_KEY` | OpenAI API key for chat, translation and level rewrites |
| `QDRANT_URL` / `QDRANT_API_KEY` | Qdrant Cloud cluster (RAG context) |
| `QDRANT_COLLECTION_NAME` | Qdrant collection name |
| `DATABASE_URL` | Neon PostgreSQL connection string (auth sessions/users) |
| `BETTER_AUTH_SECRET` | Secret for auth sessions |
| `BETTER_AUTH_URL` | Backend URL for auth callbacks (`https://physical-ai-backend-mu.vercel.app`) |

## Security

- All API keys live as Vercel environment variables (never in code).
- CORS restricts access to the book's own domains.
- Error messages don't expose sensitive configuration details.
- Auth via better-auth email/password with database sessions.

## License

MIT License — see LICENSE file for details.
