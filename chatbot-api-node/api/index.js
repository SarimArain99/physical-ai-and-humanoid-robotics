import "dotenv/config";
import express from "express";
import cors from "cors";
import OpenAI from "openai";
import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";
import pg from "pg";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
// --- Workstream C: cached AI rewrites (personalize/translate) ---
import { contentCacheKey, getCachedContent, setCachedContent, deleteCachedContent } from "../content-cache.js";
import rateLimit from "express-rate-limit";
import { createPgRateLimitStore } from "../rate-limit-store.js";
import { duplicateSignupGuard, resetMailGuard } from "../auth-guards.js";
// --- Workstream D (RAG): retrieval lives in retrieval.js; index.js only
// wires it into the two chat handlers and /api/health. ---
import { searchQdrant, buildGroundedSystemPrompt, FALLBACK_SYSTEM_PROMPT, getRagStatus, checkRagReachable } from "../retrieval.js";
const { Pool } = pg;

const app = express();
app.set("trust proxy", 1);

// Security headers + stack fingerprint removal (finding #11). The API
// only returns JSON/SSE, so the CSP is deny-by-default; the frontend
// gets its own headers from frontend/vercel.json.
app.disable("x-powered-by");
app.use((req, res, next) => {
  res.setHeader("X-Content-Type-Options", "nosniff");
  res.setHeader("X-Frame-Options", "DENY");
  res.setHeader("Referrer-Policy", "no-referrer");
  res.setHeader("Content-Security-Policy", "default-src 'none'; frame-ancestors 'none'; base-uri 'none'");
  next();
});

// CORS (finding #1, CRITICAL): allowlist ONLY the real origins. The
// frontend is same-origin in production (Vercel rewrites /api/* to this
// backend), so the only cross-origin callers are local development and
// whatever CORS_ORIGINS lists. NEVER trust a suffix such as `.vercel.app`
// with credentials — Vercel subdomains are free and attacker-controlled,
// so that rule let any malicious Vercel page read a signed-in victim's
// session token. The production frontend origin is a default here (not
// only from env) so the allowlist is correct even if CORS_ORIGINS is
// unset in Vercel.
const PRODUCTION_FRONTEND_ORIGIN = "https://physical-ai-and-humanoid-robotics-sarim.vercel.app";
const DEV_ORIGINS = [
  "http://localhost:3000",
  "http://localhost:3001",
  "http://127.0.0.1:3000",
  "http://127.0.0.1:3001",
];
const allowedOrigins = [
  ...new Set([
    PRODUCTION_FRONTEND_ORIGIN,
    ...DEV_ORIGINS,
    ...(process.env.CORS_ORIGINS
      ? process.env.CORS_ORIGINS.split(",").map((o) => o.trim().replace(/\/$/, ""))
      : []),
  ]),
];

app.use(cors({
  origin: function (origin, callback) {
    // No Origin header (curl, server-to-server, same-origin navigation)
    // is not a cross-origin browser request — nothing to reflect.
    if (!origin) return callback(null, true);
    if (allowedOrigins.includes(origin)) {
      callback(null, true);
    } else {
      // Clean rejection: no CORS headers, no error — the browser blocks
      // the cross-origin read. (callback(new Error(...)) would surface
      // as a noisy 500; the request itself is not an error.)
      callback(null, false);
    }
  },
  credentials: true,
}));
app.use(express.json());

// Provide the Better Auth API at /api/auth.
//
// Finding #2 (CRITICAL): the old /api/auth middleware REWROTE any
// `.vercel.app`/localhost Origin header to BETTER_AUTH_URL, which
// defeated better-auth's CSRF Origin check — a malicious Vercel page
// could change a signed-in victim's profile. The rewrite is DELETED
// entirely. better-auth now checks the real Origin against
// trustedOrigins (api/auth.js), which lists only the real frontend
// origin + local dev. Cross-origin auth is not needed: the frontend is
// same-origin in production (Vercel /api/* rewrite), so the session
// cookie is first-party (sameSite "Lax" in api/auth.js).

// The guards live in auth-guards.js (unit-testable without the app
// or a database). duplicateSignupGuard rewrites the duplicate-signup
// 422 into a neutral 200 (finding #6); resetMailGuard fails password
// reset honestly with a 503 when no mail transport is configured
// (finding #5). The Origin-rewrite middleware that used to sit here
// is deleted — see the finding #2 comment above.

app.use("/api/auth", duplicateSignupGuard, resetMailGuard, toNodeHandler(auth));

// Setup Neon DB Postgres Pool (created before the rate limiters —
// the shared rate-limit store below is backed by this pool).
const pool = new Pool({
  connectionString: process.env.DATABASE_URL
});

// Configure rate limits for AI endpoints to prevent API key abuse.
// Finding #7: the default in-memory store is per-process, so on Vercel
// serverless a burst spread across instances never trips the limit
// (60 rapid requests produced 0x 429 live). Both limiters below share
// counters through the Neon database (rate-limit-store.js), so the
// count is global. If the database is unavailable the store falls back
// to memory instead of failing requests. Each limiter has its own
// namespace so their counters never collide.
const AI_RATE_LIMIT_WINDOW_MS = 15 * 60 * 1000;
const aiLimiter = rateLimit({
  windowMs: AI_RATE_LIMIT_WINDOW_MS,
  max: 60, // Limit each IP to 60 requests per windowMs
  standardHeaders: true,
  legacyHeaders: false,
  store: createPgRateLimitStore(pool, { namespace: "ai", windowMs: AI_RATE_LIMIT_WINDOW_MS }),
  message: { error: "Too many AI requests. Please try again in 15 minutes." }
});

// Non-AI API endpoints (progress reads/writes, cache reads) get a
// higher budget — they are cheap and called often (every chapter/quiz),
// but they were not limited at all before.
const apiLimiter = rateLimit({
  windowMs: AI_RATE_LIMIT_WINDOW_MS,
  max: 300,
  standardHeaders: true,
  legacyHeaders: false,
  store: createPgRateLimitStore(pool, { namespace: "api", windowMs: AI_RATE_LIMIT_WINDOW_MS }),
  message: { error: "Too many requests. Please try again in a few minutes." }
});

// Apply rate limits to all OpenAI-linked endpoints
app.use("/api/chat", aiLimiter);
app.use("/api/chat/selected", aiLimiter);
app.use("/api/translate", aiLimiter);
app.use("/api/personalize", aiLimiter);
app.use("/api/quiz", aiLimiter);
app.use("/api/flashcards", aiLimiter);
app.use("/api/progress", apiLimiter);
app.use("/api/cache", apiLimiter);

const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY,
});

// Initialize DB schema
pool.query(`
  CREATE TABLE IF NOT EXISTS chat_sessions (
    id VARCHAR(255) PRIMARY KEY,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
  );
  CREATE TABLE IF NOT EXISTS chat_messages (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(255) REFERENCES chat_sessions(id),
    role VARCHAR(50),
    content TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
  );
  CREATE TABLE IF NOT EXISTS content_cache (
    key VARCHAR(255) PRIMARY KEY,
    content TEXT
  );
  CREATE TABLE IF NOT EXISTS user_progress (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL,
    page_id VARCHAR(255) NOT NULL,
    quiz_score INT,
    completed BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, page_id)
  );
  CREATE TABLE IF NOT EXISTS rate_limit (
    key VARCHAR(255) PRIMARY KEY,
    hits INT NOT NULL DEFAULT 0,
    reset_at TIMESTAMPTZ NOT NULL
  );
  -- Finding #9 (chat session binding): a session belongs to the user
  -- who created it (NULL = guest). The binding is enforced in
  -- getSessionHistory below.
  ALTER TABLE chat_sessions ADD COLUMN IF NOT EXISTS user_id VARCHAR(255);
  -- Finding #3 (cache TTL): created_at drives the 30-day expiry in
  -- content-cache.js. Existing rows get the migration time, so they get
  -- a fresh 30-day window rather than expiring at once.
  ALTER TABLE content_cache ADD COLUMN IF NOT EXISTS created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP;
`).then(() => console.log('Neon DB initialized for chatbot API.')).catch(console.error);

// Load the textbook context
let textbookContext = "";

function readAllMarkdown(dirPath) {
  let content = "";
  if (!fs.existsSync(dirPath)) return content;
  
  const files = fs.readdirSync(dirPath);
  for (const file of files) {
    const fullPath = path.join(dirPath, file);
    if (fs.statSync(fullPath).isDirectory()) {
      content += readAllMarkdown(fullPath);
    } else if (fullPath.endsWith(".md") || fullPath.endsWith(".mdx")) {
      content += `\n\n--- FILE: ${file} ---\n`;
      content += fs.readFileSync(fullPath, "utf-8");
    }
  }
  return content;
}

// The RAG context is this book's chapters (frontend/docs). Resolve it
// relative to this file, not the working directory, so it works both
// locally (chatbot-api-node/api/index.js) and on Vercel (api/index.js,
// Root Directory = chatbot-api-node). On Vercel the docs folder sits
// outside the Root Directory, so it is absent there — readAllMarkdown
// returns "" and only the /api/health contextChars diagnostic is
// affected; Qdrant does the grounding, so chat still works.
// Set TEXTBOOK_DOCS_PATH to use a different docs folder.
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const docsPath = process.env.TEXTBOOK_DOCS_PATH || path.join(__dirname, "..", "..", "frontend", "docs");

try {
  textbookContext = readAllMarkdown(docsPath);
  console.log(`Loaded textbook context (${textbookContext.length} characters).`);
} catch (error) {
  console.error("Failed to load textbook context:", error);
}

// --- Workstream D (RAG): the whole-book dump is gone. The old
// SYSTEM_PROMPT embedded the entire textbook (~434 KB) in every session
// and called that RAG — it was not retrieval, and it burned context on
// every request. Sessions now store this short instruction; each chat
// request builds its own grounded system prompt from the top-5 Qdrant
// chunks (see buildGroundedSystemPrompt in retrieval.js), or falls back
// to this prompt with grounded:false when Qdrant is unavailable.
// textbookContext is still loaded below only for the /api/health
// contextChars diagnostic — it is no longer sent to the model. ---
const SYSTEM_PROMPT = FALLBACK_SYSTEM_PROMPT;

// Finding #8 (unbounded input/history): a single request could carry
// ~100 KB and a long session re-sent its whole history to the model on
// every request — open-ended OpenAI cost. Caps below: chat messages
// 4 KB, highlighted text 8 KB, other AI text inputs 16 KB (the
// frontend sends at most ~2 KB chunks, so these never bind on normal
// use), and only the last 10 non-system messages are sent to the model.
const MAX_CHAT_MESSAGE_CHARS = 4096;
const MAX_SELECTED_TEXT_CHARS = 8192;
const MAX_AI_TEXT_CHARS = 16384;
const MAX_HISTORY_MESSAGES = 10;

function rejectOversizeText(res, value, limit, what) {
  if (typeof value !== "string" || value.length > limit) {
    res.status(413).json({ error: `${what} is too long (max ${limit} characters).` });
    return true;
  }
  return false;
}

// Finding #9 (chat session binding): resolve the signed-in user for a
// request from the better-auth session cookie. Returns the user id, or
// null for a guest (no session). Throws on a verification failure —
// callers fail the request rather than silently treating a signed-in
// user as a guest, which would hand their session to an anonymous
// caller. The chat endpoints are same-origin (Vercel /api/* rewrite),
// so the first-party cookie is present when the user is signed in.
async function getRequestUserId(req) {
  const session = await auth.api.getSession({ headers: req.headers });
  return session && session.user ? session.user.id : null;
}

// Same, but answers the HTTP request directly: 401 when there is no
// session, 500 when the session cannot be verified. Returns the user
// id, or null after answering.
async function requireUserId(req, res) {
  let userId;
  try {
    userId = await getRequestUserId(req);
  } catch (error) {
    console.error("Session verification failed:", error.message);
    res.status(500).json({ error: "Could not verify your session. Please try again." });
    return null;
  }
  if (!userId) {
    res.status(401).json({ error: "Sign in to continue." });
    return null;
  }
  return userId;
}

async function getSessionHistory(sessionId, userId = null) {
  // Finding #9: a chat session belongs to one user. The session id was
  // the only barrier — anyone who knew it could load the full history
  // into the model. Now the owner is stored on the session row and
  // enforced here:
  // - a session with an owner only serves that owner (anyone else, and
  //   any guest, gets a 403 via SESSION_OWNER_MISMATCH);
  // - a guest session (owner NULL) adopted by a newly signed-in user
  //   takes that user's id, so chatting before sign-in keeps working;
  // - a new session is created with the requester's id (NULL for guests).
  const existing = await pool.query(`SELECT user_id FROM chat_sessions WHERE id = $1`, [sessionId]);
  if (existing.rows.length > 0) {
    const owner = existing.rows[0].user_id;
    if (owner && owner !== userId) {
      const error = new Error("This chat session belongs to another account.");
      error.code = "SESSION_OWNER_MISMATCH";
      throw error;
    }
    if (owner === null && userId) {
      await pool.query(`UPDATE chat_sessions SET user_id = $1 WHERE id = $2`, [userId, sessionId]);
    }
  } else {
    await pool.query(
      `INSERT INTO chat_sessions (id, user_id) VALUES ($1, $2) ON CONFLICT DO NOTHING`,
      [sessionId, userId]
    );
  }

  const res = await pool.query(`SELECT role, content FROM chat_messages WHERE session_id = $1 ORDER BY id ASC`, [sessionId]);
  const history = res.rows.map(r => ({ role: r.role, content: r.content }));

  if (history.length === 0) {
    history.push({ role: "system", content: SYSTEM_PROMPT });
    await pool.query(`INSERT INTO chat_messages (session_id, role, content) VALUES ($1, $2, $3)`, [sessionId, "system", SYSTEM_PROMPT]);
  }
  return history;
}

async function appendMessage(sessionId, role, content) {
  await pool.query(`INSERT INTO chat_messages (session_id, role, content) VALUES ($1, $2, $3)`, [sessionId, role, content]);
}

// Load a chat session's history for this request, enforcing the owner
// binding (finding #9). Answers 500/403 itself and returns null when
// it has; otherwise returns { history }.
async function loadChatHistory(req, res, sessionId) {
  let userId;
  try {
    userId = await getRequestUserId(req);
  } catch (error) {
    console.error("Session verification failed:", error.message);
    res.status(500).json({ error: "Could not verify your session. Please try again." });
    return null;
  }
  try {
    const history = await getSessionHistory(sessionId, userId);
    return { history };
  } catch (error) {
    if (error.code === "SESSION_OWNER_MISMATCH") {
      res.status(403).json({ error: "This chat session belongs to another account." });
      return null;
    }
    console.error("Failed to load chat history:", error.message);
    res.status(500).json({ error: "Could not load the chat session. Please try again." });
    return null;
  }
}

// --- Fix round (2026-11-10): a refusal must not carry sources. ---
// The grounded prompt tells the model to say plainly that the book does
// not cover the topic when the retrieved passages do not contain the
// answer — but the meta event (with the retrieved sources) is sent
// BEFORE the answer streams, so a refusal still arrived with sources
// attached ("What is the capital of France?" listed Week 13 and Week 8
// under a refusal). The chat handlers send a trailing meta event after
// the stream finishes with the corrected sources (empty on a refusal);
// the widget keeps the last meta it saw, so the trailing event wins.
// The patterns match the refusal wording the prompts prescribe; a
// normal grounded answer never contains them.
const REFUSAL_PATTERNS = [
  /does not cover/i,
  /doesn'?t cover/i,
  /not covered in (the |this )?(book|course)/i,
  /outside (the scope of )?(the |this )?(book|course)/i,
  /can'?t answer .* from (the |this )?(book|course)/i,
];

export function isRefusalAnswer(text) {
  if (typeof text !== "string") return false;
  return REFUSAL_PATTERNS.some((re) => re.test(text));
}

// The meta event for a finished answer: sources are dropped when the
// answer is a refusal, and the refusal flag is explicit so clients do
// not have to pattern-match the text themselves. Exported for tests.
export function finalMeta(retrieval, assistantContent) {
  const refusal = isRefusalAnswer(assistantContent);
  return {
    // grounded keeps its meaning (retrieval worked), even for a refusal:
    // a refusal is a grounded non-answer, not an ungrounded one. The
    // widget hides sources on refusal and shows the ungrounded note
    // only when retrieval actually failed.
    grounded: retrieval.available,
    retrieval: retrieval.available ? "qdrant" : "fallback",
    reason: retrieval.error || null,
    sources: refusal ? [] : retrieval.sources,
    refusal,
  };
}

app.post("/api/chat", async (req, res) => {
  const { message, session_id } = req.body;
  if (!message || !session_id) {
    return res.status(400).json({ error: "Missing message or session_id" });
  }
  // Finding #8: reject oversized messages instead of paying to process them.
  if (rejectOversizeText(res, message, MAX_CHAT_MESSAGE_CHARS, "Message")) return;

  const loaded = await loadChatHistory(req, res, session_id);
  if (!loaded) return;
  const history = loaded.history;
  // --- Workstream D (RAG): retrieve the top-5 chapter chunks, then
  // answer ONLY from them. The stored session history's system message
  // is dropped here — the per-request grounded prompt replaces it, so
  // old sessions (which stored the whole-book dump) benefit too. ---
  const retrieval = await searchQdrant(openai, message);
  const messages = [
    { role: "system", content: retrieval.available ? buildGroundedSystemPrompt(retrieval.sources) : FALLBACK_SYSTEM_PROMPT },
    // Finding #8: only the most recent exchanges go to the model — a
    // long session no longer re-sends its whole history every request.
    ...history.filter((m) => m.role !== "system").slice(-MAX_HISTORY_MESSAGES),
    { role: "user", content: message },
  ];
  await appendMessage(session_id, "user", message);

  try {
    res.setHeader("Content-Type", "text/event-stream");
    res.setHeader("Cache-Control", "no-cache");
    res.setHeader("Connection", "keep-alive");

    // First event is metadata, not answer text: grounded true/false,
    // why (when falling back), and the sources the widget shows under
    // the answer. The widget ignores it if it only reads content.
    res.write(`data: ${JSON.stringify({ meta: { grounded: retrieval.available, retrieval: retrieval.available ? "qdrant" : "fallback", reason: retrieval.error || null, sources: retrieval.sources } })}\n\n`);

    const stream = await openai.chat.completions.create({
      model: "gpt-4o-mini",
      messages,
      stream: true,
    });

    let assistantContent = "";

    for await (const chunk of stream) {
      const content = chunk.choices[0]?.delta?.content || "";
      if (content) {
        assistantContent += content;
        res.write(`data: ${JSON.stringify({ content })}\n\n`);
      }
    }

    await appendMessage(session_id, "assistant", assistantContent);
    // Fix round (2026-11-10): trailing meta with the corrected sources
    // (empty when the answer is a refusal — see finalMeta above). The
    // widget keeps the last meta event, so this overwrites the one sent
    // before the stream.
    res.write(`data: ${JSON.stringify({ meta: finalMeta(retrieval, assistantContent) })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
  } catch (error) {
    console.error(error);
    res.write(`data: ${JSON.stringify({ content: "Error connecting to AI." })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
  }
});

app.post("/api/chat/selected", async (req, res) => {
  const { message, selected_text, session_id } = req.body;
  if (!message || !selected_text || !session_id) {
    return res.status(400).json({ error: "Missing message, selected_text, or session_id" });
  }
  // Finding #8: same caps as /api/chat (the highlight is chapter text,
  // so it gets a larger budget than the question itself).
  if (rejectOversizeText(res, message, MAX_CHAT_MESSAGE_CHARS, "Message")) return;
  if (rejectOversizeText(res, selected_text, MAX_SELECTED_TEXT_CHARS, "Selected text")) return;

  const loaded = await loadChatHistory(req, res, session_id);
  if (!loaded) return;
  const history = loaded.history;
  const prompt = `The user has highlighted the following text from the textbook:\n"${selected_text}"\n\nUser's Question:\n${message}`;

  // --- Workstream D (RAG): same retrieval as /api/chat, but the query
  // is the question + the highlighted text, and the highlighted text is
  // the primary context in the grounded prompt. ---
  const retrieval = await searchQdrant(openai, `${message}\n${selected_text}`);
  const messages = [
    { role: "system", content: retrieval.available ? buildGroundedSystemPrompt(retrieval.sources, { selectedText: selected_text }) : FALLBACK_SYSTEM_PROMPT },
    // Finding #8: only the most recent exchanges go to the model.
    ...history.filter((m) => m.role !== "system").slice(-MAX_HISTORY_MESSAGES),
    { role: "user", content: prompt },
  ];
  await appendMessage(session_id, "user", prompt);

  try {
    res.setHeader("Content-Type", "text/event-stream");
    res.setHeader("Cache-Control", "no-cache");
    res.setHeader("Connection", "keep-alive");

    // Metadata first (see /api/chat): grounded flag, reason, sources.
    res.write(`data: ${JSON.stringify({ meta: { grounded: retrieval.available, retrieval: retrieval.available ? "qdrant" : "fallback", reason: retrieval.error || null, sources: retrieval.sources } })}\n\n`);

    const stream = await openai.chat.completions.create({
      model: "gpt-4o-mini",
      messages,
      stream: true,
    });

    let assistantContent = "";

    for await (const chunk of stream) {
      const content = chunk.choices[0]?.delta?.content || "";
      if (content) {
        assistantContent += content;
        res.write(`data: ${JSON.stringify({ content })}\n\n`);
      }
    }

    await appendMessage(session_id, "assistant", assistantContent);
    // Fix round (2026-11-10): trailing meta with the corrected sources
    // (empty when the answer is a refusal — see finalMeta above).
    res.write(`data: ${JSON.stringify({ meta: finalMeta(retrieval, assistantContent) })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
  } catch (error) {
    console.error(error);
    res.write(`data: ${JSON.stringify({ content: "Error connecting to AI." })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
  }
});

// Batched AI requests join several passages with the [[[BREAK]]] marker.
// The model MUST keep every marker exactly as-is, in order, one rewritten
// passage per input passage. If it merges passages or drops markers, the
// frontend cannot split the reply back up and the whole batch is lost
// (this is what broke the "Intermediate" level: its expanding rewrite
// dropped the markers, so every chunk came back as one un-splittable
// passage and the page silently fell back to the original text).
const BATCH_DELIMITER_INSTRUCTION =
  "The input contains several passages separated by the exact marker [[[BREAK]]]. " +
  "Rewrite each passage separately and keep every [[[BREAK]]] marker exactly as-is, " +
  "in the same order, with nothing added or removed. Do not merge passages and " +
  "do not add or remove markers.";

// --- Workstream C: cached AI rewrites (personalize/translate) ---
// Root cause of the ~7-minute Level rewrite: the frontend sent every
// chunk sequentially AND the backend never used the content_cache table,
// so each visit re-paid a full OpenAI call per chunk (and, when the
// model dropped the [[[BREAK]]] markers, one call per paragraph).
// This helper serves a cached chunk as SSE (same shape as a live
// stream, so the frontend needs no special case) and stores a fresh
// rewrite for the next visit. The cache never fails the request: a
// DB error or timeout degrades to a normal OpenAI call. Only a
// completed, non-empty stream is cached — the "Error connecting to AI"
// text is never stored. One request still does one chunk only; the
// frontend fans chunks out concurrently instead (see runChunked in
// frontend/src/config/api.js), so no Vercel invocation does minutes
// of work. X-Content-Cache reports HIT/MISS for debugging.
// --- End Workstream C ---
async function streamCachedOrGenerate(req, res, { kind, variant, page, text, systemPrompt, errorText, maxTokens = 4096, postProcess = null }) {
  const key = contentCacheKey({ kind, variant, page, text });
  const cached = await getCachedContent(pool, key);

  res.setHeader("Content-Type", "text/event-stream");
  res.setHeader("Cache-Control", "no-cache");
  res.setHeader("Connection", "keep-alive");

  if (cached !== null) {
    res.setHeader("X-Content-Cache", "HIT");
    res.write(`data: ${JSON.stringify({ content: cached })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
    return;
  }

  res.setHeader("X-Content-Cache", "MISS");
  try {
    const stream = await openai.chat.completions.create({
      model: "gpt-4o-mini",
      messages: [
        { role: "system", content: systemPrompt },
        { role: "user", content: text },
      ],
      stream: true,
      temperature: 0.3,
      max_tokens: maxTokens,
    });

    let fullText = "";
    for await (const chunk of stream) {
      const content = chunk.choices[0]?.delta?.content || "";
      if (content) {
        fullText += content;
        // A post-process step (the translate untranslated-retry) needs
        // the finished text before it can judge it, so validated kinds
        // buffer instead of streaming deltas — the reply is still one
        // SSE content event plus [DONE], exactly the shape the
        // frontend reads, and progress still counts finished chunks.
        if (!postProcess) res.write(`data: ${JSON.stringify({ content })}\n\n`);
      }
    }

    if (postProcess) {
      try {
        fullText = await postProcess(text, fullText);
      } catch (error) {
        console.error("Post-process failed, keeping the generated text:", error.message);
      }
      res.write(`data: ${JSON.stringify({ content: fullText })}\n\n`);
    }

    // Fire-and-forget is unsafe on serverless (code after res.end may
    // not run), so await the write — it is capped at 2s inside the
    // helper and never throws, so it cannot stall or fail the reply.
    if (fullText.trim()) await setCachedContent(pool, key, fullText);

    res.write("data: [DONE]\n\n");
    res.end();
  } catch (error) {
    console.error(error);
    res.write(`data: ${JSON.stringify({ content: errorText })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
  }
}

// --- Translation quality (2026-11-10): bold + Hindi script ---
// Live defects on Week 2 (French + Hindi): the model dropped the
// **bold** markers on list-item lead-ins, left headings in English,
// and wrote a Hindi paragraph in Roman-script Hinglish. The prompt
// below is the first fix; the frontend re-applies a dropped lead-in
// bold as a safety net (see restoreLeadInBold in
// frontend/src/utils/markdown.js), and validateTranslationResult
// retries a chunk that came back untranslated.
//
// Extra instructions for languages with a known failure mode. Hindi
// is the only one so far; the base prompt already says to translate
// everything, and this makes the script requirement explicit.
const TRANSLATE_LANGUAGE_NOTES = {
  Hindi:
    "For Hindi: write the entire translation in Devanagari script. Do not use Roman (Latin) script for Hindi words — transliterated Hinglish written in Latin letters is not acceptable. Translate every heading into Devanagari as well. Only established technical terms and product names (ROS, ROS 2, LiDAR, AI, Gazebo, Python) stay in English in Latin script.",
};

// The translate system prompt. Exported for tests: the prompt is part
// of what a cached translation depends on, so a change here must be
// paired with the translate cache-key version bump (see
// content-cache.js — translate is v2 since this prompt landed).
export function buildTranslatePrompt(targetLanguage) {
  const langNote = TRANSLATE_LANGUAGE_NOTES[targetLanguage] || "";
  return (
    `You are a professional technical translator. Translate the following educational text into natural, accurate ${targetLanguage}. ` +
    `Translate EVERY passage, including every heading, list item and caption — do not leave any passage in the original language. ` +
    `Preserve all formatting and markdown structure: keep **bold** markers exactly where they are in the source, especially the bold lead-in at the start of a list item (for example **The agent's body:** stays bold in the translation, in the target language). ` +
    `Keep technical terms (ROS, ROS 2, LiDAR, AI, Gazebo, Python) in English when they have no common translation in the target language. ` +
    (langNote ? langNote + " " : "") +
    `Provide ONLY the translated text, without any introductory conversational filler. ` +
    BATCH_DELIMITER_INSTRUCTION
  );
}

// Split a batched chunk back into its passages (mirrors the frontend's
// split in frontend/src/config/api.js).
export function splitBatchText(text) {
  if (typeof text !== "string") return [];
  return text
    .split("[[[BREAK]]]")
    .map((part) => part.trim())
    .filter((part) => part.length > 0);
}

// Indices of passages the model left untranslated (output identical
// to input). Short passages (a bare term, a number) are often
// identical by nature, so only real prose counts. A count mismatch
// means the passages cannot be aligned safely — nothing is flagged.
export function findUntranslatedPassages(inputParts, outputParts) {
  if (!Array.isArray(inputParts) || !Array.isArray(outputParts)) return [];
  if (inputParts.length !== outputParts.length) return [];
  const found = [];
  inputParts.forEach((input, i) => {
    const source = (input || "").trim();
    const output = (outputParts[i] || "").trim();
    if (source.length > 15 && source === output) found.push(i);
  });
  return found;
}

// One retry for a chunk that came back (partly) untranslated.
// retryOnce is called at most once and only when a passage was left
// untranslated; its result replaces the output only when it splits
// into the same number of passages (a merged reply would misalign
// the page). A retry that still leaves English in place, or any
// failure, keeps the original output — the frontend leaves the
// English for passages it cannot use, which beats a wrong
// translation. Exported for tests.
export async function validateTranslationResult({ inputText, outputText, retryOnce }) {
  if (typeof outputText !== "string" || !outputText.trim()) return outputText;
  const inputParts = splitBatchText(inputText);
  const outputParts = splitBatchText(outputText);
  if (inputParts.length === 0 || inputParts.length !== outputParts.length) return outputText;
  if (findUntranslatedPassages(inputParts, outputParts).length === 0) return outputText;
  let retried = null;
  try {
    retried = await retryOnce();
  } catch {
    return outputText;
  }
  if (typeof retried !== "string" || !retried.trim()) return outputText;
  if (splitBatchText(retried).length !== inputParts.length) return outputText;
  return retried;
}

app.post("/api/translate", async (req, res) => {
  const { text, target_language = "Urdu", page } = req.body;

  if (!text) {
    return res.status(400).json({ error: "Missing text to translate" });
  }
  // Finding #8: the frontend sends at most ~2 KB chunks; anything far
  // larger is abuse, not a chapter passage.
  if (rejectOversizeText(res, text, MAX_AI_TEXT_CHARS, "Text")) return;

  const systemPrompt = buildTranslatePrompt(target_language);

  await streamCachedOrGenerate(req, res, {
    kind: "translate",
    variant: target_language,
    page,
    text,
    systemPrompt,
    errorText: "Error connecting to AI for translation.",
    // Retry a chunk that came back (partly) untranslated, once, with
    // a stricter prompt — see validateTranslationResult above.
    postProcess: async (inputText, outputText) =>
      validateTranslationResult({
        inputText,
        outputText,
        retryOnce: async () => {
          const retry = await openai.chat.completions.create({
            model: "gpt-4o-mini",
            messages: [
              {
                role: "system",
                content:
                  buildTranslatePrompt(target_language) +
                  ` One or more passages in the previous attempt came back untranslated. Translate EVERY passage, including every heading, into ${target_language}. Do not leave any passage in the original language.`,
              },
              { role: "user", content: inputText },
            ],
            stream: false,
            temperature: 0.3,
            max_tokens: 4096,
          });
          return retry.choices[0]?.message?.content || "";
        },
      }),
  });
});

app.post("/api/personalize", async (req, res) => {
  const { text, level, page } = req.body;

  if (!text || !level) {
    return res.status(400).json({ error: "Missing text or level" });
  }
  // Finding #8: same cap as /api/translate.
  if (rejectOversizeText(res, text, MAX_AI_TEXT_CHARS, "Text")) return;

  let systemPrompt = "";
  if (level === "beginner") {
    systemPrompt = "You are a professional technical educator. Rewrite the following educational text in very simple, beginner-friendly language. Keep the core meaning accurate, remove unnecessary complexity, and explain difficult terminology in simple words. Return ONLY the rewritten text, preserving markdown formatting where appropriate. Do not add introductory filler. " + BATCH_DELIMITER_INSTRUCTION;
  } else if (level === "intermediate") {
    systemPrompt = "You are a professional technical educator. Rewrite the following educational text to provide a more detailed and clearer explanation than the original. Add useful context, clarification, and practical examples where appropriate. Maintain factual accuracy and original meaning. Return ONLY the rewritten text, preserving markdown formatting where appropriate. Do not add introductory filler. " + BATCH_DELIMITER_INSTRUCTION;
  } else {
    return res.status(400).json({ error: "Invalid level" });
  }

  // Fix round (2026-11-10): max_tokens 1500 for a level rewrite. A
  // chunk now holds at most 6 short passages (~900 chars in); a
  // rewrite of that is a few hundred tokens, so 1500 is ~3x headroom
  // and truncation is not a risk. The old 4096 let a runaway output
  // stream for over a minute — past Vercel's function limit — which
  // killed the request mid-stream and forced the expensive fallback.
  // Translate keeps the 4096 default (its behaviour is unchanged).
  await streamCachedOrGenerate(req, res, {
    kind: "personalize",
    variant: level,
    page,
    text,
    systemPrompt,
    errorText: "Error connecting to AI for personalization.",
    maxTokens: 1500,
  });
});

app.post("/api/quiz", async (req, res) => {
  const { text } = req.body;
  if (!text) return res.status(400).json({ error: "Missing text" });
  // Finding #8: same cap as the other AI text endpoints.
  if (rejectOversizeText(res, text, MAX_AI_TEXT_CHARS, "Text")) return;

  const systemPrompt = `You are a technical educator. Generate a 3-question multiple-choice quiz based on the provided text. Return ONLY a valid JSON object with a "questions" array. Format exactly like this:
{
  "questions": [
    {
      "question": "What is ROS 2?",
      "options": ["An OS", "A middleware", "A robot", "A programming language"],
      "answerIndex": 1
    }
  ]
}`;

  try {
    const completion = await openai.chat.completions.create({
      model: "gpt-4o-mini",
      messages: [
        { role: "system", content: systemPrompt },
        { role: "user", content: text }
      ],
      temperature: 0.2,
      response_format: { type: "json_object" } 
    });
    res.json({ content: completion.choices[0].message.content });
  } catch (err) {
    res.status(500).json({ error: "Failed to generate quiz" });
  }
});

app.post("/api/flashcards", async (req, res) => {
  const { text } = req.body;
  if (!text) return res.status(400).json({ error: "Missing text" });
  // Finding #8: same cap as the other AI text endpoints.
  if (rejectOversizeText(res, text, MAX_AI_TEXT_CHARS, "Text")) return;

  const systemPrompt = `You are a technical educator. Summarize the provided text into exactly 5 concise, high-yield bullet points for exam review (Flashcards). Use markdown bullet points. Return ONLY the bullet points, no introductory filler.`;

  try {
    res.setHeader("Content-Type", "text/event-stream");
    res.setHeader("Cache-Control", "no-cache");
    res.setHeader("Connection", "keep-alive");

    const stream = await openai.chat.completions.create({
      model: "gpt-4o-mini",
      messages: [
        { role: "system", content: systemPrompt },
        { role: "user", content: text }
      ],
      stream: true,
      temperature: 0.2,
    });

    for await (const chunk of stream) {
      const content = chunk.choices[0]?.delta?.content || "";
      if (content) res.write(`data: ${JSON.stringify({ content })}\n\n`);
    }
    res.write("data: [DONE]\n\n");
    res.end();
  } catch (err) {
    res.write(`data: ${JSON.stringify({ content: "Error generating flashcards." })}\n\n`);
    res.write("data: [DONE]\n\n");
    res.end();
  }
});

// Progress Endpoints
// Finding #4 (IDOR): these took user_id from the request with no
// session check at all — anyone could read any student's progress and
// fabricate a 100 for any account. Now the user id comes ONLY from the
// better-auth session; the request body/path never supplies it. (The
// frontend keeps progress in localStorage and never calls these
// endpoints, so requiring a session breaks no existing flow.)
app.post("/api/progress", async (req, res) => {
  try {
    const userId = await requireUserId(req, res);
    if (!userId) return;
    const { page_id, quiz_score, completed } = req.body;
    // A mismatched user_id in the body is a cross-user write attempt:
    // reject it explicitly instead of silently writing to the session user.
    if (req.body.user_id && req.body.user_id !== userId) {
      return res.status(403).json({ error: "You can only update your own progress." });
    }
    if (!page_id) return res.status(400).json({ error: "Missing page_id" });

    await pool.query(
      `INSERT INTO user_progress (user_id, page_id, quiz_score, completed) 
       VALUES ($1, $2, $3, $4) 
       ON CONFLICT (user_id, page_id) 
       DO UPDATE SET 
          quiz_score = GREATEST(user_progress.quiz_score, EXCLUDED.quiz_score),
          completed = EXCLUDED.completed`,
      [userId, page_id, quiz_score || 0, completed || true]
    );
    res.json({ success: true });
  } catch (err) {
    console.error(err);
    res.status(500).json({ error: "Database error" });
  }
});

app.get("/api/progress/:userId", async (req, res) => {
  try {
    const userId = await requireUserId(req, res);
    if (!userId) return;
    // A user can only ever read their own progress, even though the id
    // is in the path.
    if (req.params.userId !== userId) {
      return res.status(403).json({ error: "You can only view your own progress." });
    }
    const result = await pool.query(`SELECT page_id, quiz_score, completed, created_at FROM user_progress WHERE user_id = $1 ORDER BY created_at DESC`, [userId]);
    res.setHeader("Cache-Control", "no-store, no-cache, must-revalidate, proxy-revalidate");
    res.json({ progress: result.rows });
  } catch (err) {
    res.status(500).json({ error: "Database error" });
  }
});

// Cache Endpoints — thin wrappers over the shared Workstream C helpers
// in content-cache.js, so there is one code path for the table. Unlike
// the translate/personalize handlers (where a cache failure must degrade
// to a normal AI call), these endpoints keep their old contract: a DB
// error is a 500, a missing key is a 404.
//
// Finding #3 (cache poisoning, CRITICAL): the old public POST
// /api/cache let anyone upsert any key with no auth — an attacker
// pre-seeded poisoned translations that were then served to every
// reader as a cache HIT. The POST endpoint is DELETED entirely: the
// translate/personalize handlers already write the cache server-side,
// so no client ever needs to write it. What remains is a read (the
// same content any reader gets through /api/translate) and an
// admin-only delete for invalidation.
app.get("/api/cache/:key", async (req, res) => {
  try {
    const content = await getCachedContent(pool, req.params.key, { throwOnError: true });
    if (content !== null) {
      res.json({ content });
    } else {
      res.status(404).json({ error: "Cache miss" });
    }
  } catch (err) {
    res.status(500).json({ error: "Database error" });
  }
});

// Admin-only cache invalidation (finding #3): delete one cached entry
// by key. Requires a signed-in session whose user holds the admin role
// (better-auth admin plugin). No user holds that role today, so this
// is locked until the owner assigns it — by design, it must never be
// open to ordinary accounts.
app.delete("/api/cache/:key", async (req, res) => {
  try {
    let session = null;
    try {
      session = await auth.api.getSession({ headers: req.headers });
    } catch (error) {
      console.error("Session verification failed:", error.message);
      return res.status(500).json({ error: "Could not verify your session. Please try again." });
    }
    if (!session || !session.user) {
      return res.status(401).json({ error: "Sign in to continue." });
    }
    if (session.user.role !== "admin") {
      return res.status(403).json({ error: "Only an admin can invalidate cached content." });
    }
    const deleted = await deleteCachedContent(pool, req.params.key, { throwOnError: true });
    if (!deleted) return res.status(404).json({ error: "Cache miss" });
    res.json({ success: true });
  } catch (err) {
    res.status(500).json({ error: "Database error" });
  }
});

// Health check endpoint
// contextChars reports how much textbook text the RAG context holds —
// 0 means the docs were not found at startup (check the docs path).
app.get("/api/health", async (req, res) => {
  // Key presence only — never the value. "missing" explains at a glance
  // why every AI endpoint returns "Error connecting to AI.".
  const ragStatus = getRagStatus();
  // Live reachability, not just env presence: "configured" only says
  // the vars exist — the chat can still fall back with "fetch failed"
  // when the cluster itself is unreachable (wrong URL, paused or
  // deleted cluster, network/TLS failure). Probe once so health
  // reports what the chat will actually do. Nothing on this path is
  // used by the frontend on page load, so a slow probe only slows a
  // diagnostic call.
  let reachable = null;
  let reachableError = null;
  if (ragStatus.qdrant === "configured") {
    const probe = await checkRagReachable();
    reachable = probe.reachable;
    reachableError = probe.reason;
  }
  res.json({
    status: "ok",
    contextChars: textbookContext.length,
    openai: process.env.OPENAI_API_KEY ? "configured" : "missing",
    model: "gpt-4o-mini",
    // --- Workstream D (RAG): qdrant "configured" only when BOTH
    // QDRANT_URL and QDRANT_API_KEY are set; "missing" lists which are
    // absent. No values are ever returned. `reachable` is the live
    // probe: false + reachableError names why retrieval fails. ---
    rag: { ...ragStatus, reachable, reachableError },
  });
});

// NOTE: the old /api/diagnostics endpoint was removed on purpose.
// It returned masked prefixes of live secrets (DATABASE_URL,
// OPENAI_API_KEY, BETTER_AUTH_SECRET). A debug endpoint has no place
// in production — use /api/health for status checks instead.

const PORT = process.env.PORT || 8000;
if (!process.env.VERCEL) {
  app.listen(PORT, () => {
    console.log(`Node Chatbot API running on port ${PORT}`);
    // --- Workstream D (RAG): fail loudly at startup when Qdrant is not
    // configured — the chat still works, but ungrounded (see the
    // fallback in the chat handlers). Never log a URL or a key. ---
    const ragStatus = getRagStatus();
    if (ragStatus.qdrant === "configured") {
      console.log(`RAG (Qdrant): configured — collection "${ragStatus.collection}", top-${ragStatus.topK}, ${ragStatus.embeddingModel}.`);
    } else {
      console.error(`RAG (Qdrant): MISSING ${ragStatus.missing.join(" and ")} — chat will answer WITHOUT chapter retrieval (fallback mode). Set QDRANT_URL and QDRANT_API_KEY in the backend's Vercel env vars, then run node scripts/ingest.js.`);
    }
  });
}

export default app;
