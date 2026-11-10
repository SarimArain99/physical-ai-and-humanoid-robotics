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
import { contentCacheKey, getCachedContent, setCachedContent } from "./content-cache.js";
import rateLimit from "express-rate-limit";
// --- Workstream D (RAG): retrieval lives in retrieval.js; index.js only
// wires it into the two chat handlers and /api/health. ---
import { searchQdrant, buildGroundedSystemPrompt, FALLBACK_SYSTEM_PROMPT, getRagStatus } from "./retrieval.js";
const { Pool } = pg;

const app = express();
app.set("trust proxy", 1);
const allowedOrigins = process.env.CORS_ORIGINS 
  ? process.env.CORS_ORIGINS.split(",").map(o => o.trim()) 
  : [];

app.use(cors({
  origin: function (origin, callback) {
    if (!origin) return callback(null, true);
    
    const isAllowed = allowedOrigins.includes(origin) || 
      origin.startsWith('http://localhost') || 
      origin.startsWith('http://127.0.0.1') || 
      origin.endsWith('.vercel.app');
      
    if (isAllowed) {
      callback(null, true);
    } else {
      callback(new Error('Not allowed by CORS'));
    }
  },
  credentials: true,
}));
app.use(express.json());

// Provide the Better Auth API at /api/auth
app.use("/api/auth", (req, res, next) => {
  const origin = req.headers.origin;
  if (origin) {
    const isAllowed = origin.startsWith('http://localhost') || 
                      origin.startsWith('http://127.0.0.1') || 
                      origin.endsWith('.vercel.app');
    if (isAllowed && process.env.BETTER_AUTH_URL) {
      // Normalize and match BETTER_AUTH_URL to pass Better Auth CSRF
      req.headers.origin = process.env.BETTER_AUTH_URL.replace(/\/$/, "");
    }
  }
  next();
}, toNodeHandler(auth));

// Configure rate limits for AI endpoints to prevent API key abuse
const aiLimiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 60, // Limit each IP to 60 requests per windowMs
  standardHeaders: true,
  legacyHeaders: false,
  message: { error: "Too many AI requests. Please try again in 15 minutes." }
});

// Apply rate limits to all OpenAI-linked endpoints
app.use("/api/chat", aiLimiter);
app.use("/api/chat/selected", aiLimiter);
app.use("/api/translate", aiLimiter);
app.use("/api/personalize", aiLimiter);
app.use("/api/quiz", aiLimiter);
app.use("/api/flashcards", aiLimiter);

const openai = new OpenAI({
  apiKey: process.env.OPENAI_API_KEY,
});

// Setup Neon DB Postgres Pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL
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
// locally (chatbot-api-node/index.js) and on Vercel (api/index.js).
// Set TEXTBOOK_DOCS_PATH to use a different docs folder.
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const docsPath = process.env.TEXTBOOK_DOCS_PATH || path.join(__dirname, "..", "frontend", "docs");

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

async function getSessionHistory(sessionId) {
  // Ensure session exists
  await pool.query(`INSERT INTO chat_sessions (id) VALUES ($1) ON CONFLICT DO NOTHING`, [sessionId]);
  
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

  const history = await getSessionHistory(session_id);
  // --- Workstream D (RAG): retrieve the top-5 chapter chunks, then
  // answer ONLY from them. The stored session history's system message
  // is dropped here — the per-request grounded prompt replaces it, so
  // old sessions (which stored the whole-book dump) benefit too. ---
  const retrieval = await searchQdrant(openai, message);
  const messages = [
    { role: "system", content: retrieval.available ? buildGroundedSystemPrompt(retrieval.sources) : FALLBACK_SYSTEM_PROMPT },
    ...history.filter((m) => m.role !== "system"),
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

  const history = await getSessionHistory(session_id);
  const prompt = `The user has highlighted the following text from the textbook:\n"${selected_text}"\n\nUser's Question:\n${message}`;

  // --- Workstream D (RAG): same retrieval as /api/chat, but the query
  // is the question + the highlighted text, and the highlighted text is
  // the primary context in the grounded prompt. ---
  const retrieval = await searchQdrant(openai, `${message}\n${selected_text}`);
  const messages = [
    { role: "system", content: retrieval.available ? buildGroundedSystemPrompt(retrieval.sources, { selectedText: selected_text }) : FALLBACK_SYSTEM_PROMPT },
    ...history.filter((m) => m.role !== "system"),
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
async function streamCachedOrGenerate(req, res, { kind, variant, page, text, systemPrompt, errorText, maxTokens = 4096 }) {
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
        res.write(`data: ${JSON.stringify({ content })}\n\n`);
      }
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

app.post("/api/translate", async (req, res) => {
  const { text, target_language = "Urdu", page } = req.body;

  if (!text) {
    return res.status(400).json({ error: "Missing text to translate" });
  }

  const systemPrompt = `You are a professional technical translator. Translate the following educational text into natural, accurate ${target_language}. Preserve all formatting, technical terms (keep them in English if they don't have a direct common translation), and markdown structure. Provide ONLY the translated text, without any introductory conversational filler. ${BATCH_DELIMITER_INSTRUCTION}`;

  await streamCachedOrGenerate(req, res, {
    kind: "translate",
    variant: target_language,
    page,
    text,
    systemPrompt,
    errorText: "Error connecting to AI for translation.",
  });
});

app.post("/api/personalize", async (req, res) => {
  const { text, level, page } = req.body;

  if (!text || !level) {
    return res.status(400).json({ error: "Missing text or level" });
  }

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
app.post("/api/progress", async (req, res) => {
  try {
    const { user_id, page_id, quiz_score, completed } = req.body;
    if (!user_id || !page_id) return res.status(400).json({ error: "Missing user_id or page_id" });

    await pool.query(
      `INSERT INTO user_progress (user_id, page_id, quiz_score, completed) 
       VALUES ($1, $2, $3, $4) 
       ON CONFLICT (user_id, page_id) 
       DO UPDATE SET 
          quiz_score = GREATEST(user_progress.quiz_score, EXCLUDED.quiz_score),
          completed = EXCLUDED.completed`,
      [user_id, page_id, quiz_score || 0, completed || true]
    );
    res.json({ success: true });
  } catch (err) {
    console.error(err);
    res.status(500).json({ error: "Database error" });
  }
});

app.get("/api/progress/:userId", async (req, res) => {
  try {
    const userId = req.params.userId;
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

app.post("/api/cache", async (req, res) => {
  try {
    const { key, content } = req.body;
    if (!key || !content) return res.status(400).json({ error: "Missing key or content" });

    const stored = await setCachedContent(pool, key, content, { throwOnError: true });
    if (!stored) return res.status(500).json({ error: "Database error" });
    res.json({ success: true });
  } catch (err) {
    res.status(500).json({ error: "Database error" });
  }
});

// Health check endpoint
// contextChars reports how much textbook text the RAG context holds —
// 0 means the docs were not found at startup (check the docs path).
app.get("/api/health", (req, res) => {
  // Key presence only — never the value. "missing" explains at a glance
  // why every AI endpoint returns "Error connecting to AI.".
  res.json({
    status: "ok",
    contextChars: textbookContext.length,
    openai: process.env.OPENAI_API_KEY ? "configured" : "missing",
    model: "gpt-4o-mini",
    // --- Workstream D (RAG): qdrant "configured" only when BOTH
    // QDRANT_URL and QDRANT_API_KEY are set; "missing" lists which are
    // absent. No values are ever returned. ---
    rag: getRagStatus(),
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
