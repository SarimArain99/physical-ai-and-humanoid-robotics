/**
 * Backend API Configuration
 *
 * The book's backend is the Express API at physical-ai-backend-mu.vercel.app
 * (better-auth email/password, AI chat, translation and level adjustment).
 * The old Hugging Face backend is gone and must not be used.
 *
 * NOTE: Docusaurus does not expose process.env to browser code, so the
 * production backend URL is the default. Local development uses localhost.
 */

function resolveApiBaseUrl() {
  if (typeof window !== "undefined") {
    const host = window.location.hostname;
    if (host === "localhost" || host === "127.0.0.1") {
      return "http://localhost:8000";
    }
  }
  return "https://physical-ai-backend-mu.vercel.app";
}

export const API_BASE_URL = resolveApiBaseUrl();

// Auth endpoints (better-auth protocol — the client in src/auth/client.ts
// appends /api/auth itself; these helpers are for direct calls).
export const authUrls = {
  getSession: () => `${API_BASE_URL}/api/auth/get-session`,
  signInEmail: () => `${API_BASE_URL}/api/auth/sign-in/email`,
  signUp: () => `${API_BASE_URL}/api/auth/sign-up/email`,
};

// Chat endpoints (Express API — Server-Sent Events streams)
export const chatUrls = {
  chat: () => `${API_BASE_URL}/api/chat`,
  chatSelected: () => `${API_BASE_URL}/api/chat/selected`,
};

// Content endpoints (Express API — Server-Sent Events streams)
export const contentUrls = {
  translate: () => `${API_BASE_URL}/api/translate`,
  personalize: () => `${API_BASE_URL}/api/personalize`,
};

// The Express API sends back plain error text inside a 200 SSE stream when
// the AI call itself fails. Treat those as failures, not content.
const STREAM_ERROR_SNIPPETS = [
  "Error connecting to AI.",
  "Error connecting to AI for translation.",
  "Error connecting to AI for personalization.",
];

/**
 * Read a Server-Sent Events response and return the full streamed text.
 * Calls onChunk(textSoFar) as chunks arrive so the UI can stream live.
 * Calls onMeta(meta) once per `meta` event — the chat endpoints send one
 * first (Workstream D: RAG grounded flag, retrieval mode and sources)
 * before any content event. Throws on HTTP errors or when the stream
 * carries an AI error message.
 */
export async function readSseStream(response, onChunk, onMeta) {
  if (!response.ok) {
    let detail = `Request failed (${response.status})`;
    try {
      const data = await response.json();
      if (data && data.error) detail = data.error;
    } catch {
      // Keep the generic message.
    }
    throw new Error(detail);
  }

  if (!response.body || typeof response.body.getReader !== "function") {
    // Fallback for environments without a streaming reader.
    const text = await response.text();
    return text;
  }

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  let fullText = "";

  const handleEvent = (rawEvent) => {
    for (const line of rawEvent.split("\n")) {
      const trimmed = line.trim();
      if (!trimmed.startsWith("data:")) continue;
      const payload = trimmed.slice(5).trim();
      if (payload === "[DONE]") return;
      try {
        const data = JSON.parse(payload);
        if (data.meta && typeof data.meta === "object") {
          // Retrieval metadata (chat only): grounded flag + sources.
          if (onMeta) onMeta(data.meta);
        } else if (typeof data.content === "string" && data.content) {
          fullText += data.content;
          if (onChunk) onChunk(fullText);
        }
      } catch {
        // Ignore malformed SSE lines.
      }
    }
  };

  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    buffer += decoder.decode(value, { stream: true });
    let boundary = buffer.indexOf("\n\n");
    while (boundary !== -1) {
      handleEvent(buffer.slice(0, boundary));
      buffer = buffer.slice(boundary + 2);
      boundary = buffer.indexOf("\n\n");
    }
  }
  if (buffer.trim()) handleEvent(buffer);

  if (STREAM_ERROR_SNIPPETS.some((snippet) => fullText.trim() === snippet)) {
    throw new Error("The AI service is unavailable right now. Please try again in a moment.");
  }
  return fullText;
}

// Delimiter used to translate/adjust several page elements in one API call
// (the backend only translates one text per call, and it rate-limits AI
// requests, so sending every paragraph separately would run out fast).
const CHUNK_DELIMITER = "[[[BREAK]]]";

function groupTexts(texts, maxChars = 1500) {
  const chunks = [];
  let current = [];
  let currentLen = 0;
  for (const text of texts) {
    if (current.length > 0 && currentLen + text.length > maxChars) {
      chunks.push(current);
      current = [];
      currentLen = 0;
    }
    current.push(text);
    currentLen += text.length;
  }
  if (current.length > 0) chunks.push(current);
  return chunks;
}

import { runWithConcurrency } from "../utils/runWithConcurrency";

// Workstream C — Level rewrite speed: chunks run CONCURRENTLY (bounded),
// not one after another. The old loop awaited each chunk in turn, and on
// a delimiter mismatch retried every paragraph in turn too — up to ~35
// sequential OpenAI calls for one long chapter (~7 minutes live).
// Bound: 4 in flight. Why 4: Week 1 is 6 chunks (2 waves); the longest
// chapter is ~30 chunks; 4 stays under the browser's 6-connections-per-
// origin limit and under the backend's 60-requests-per-15-minutes AI
// rate limit, while cutting wall-clock time to roughly a quarter.
// One request still does one chunk only, so no Vercel invocation does
// minutes of work; the backend caches each chunk in Neon, so a repeat
// visit is served from cache. Progress is honest counts: onProgress
// receives (done, total) — texts actually finished — never a percentage
// invented from elapsed time. Aborting the signal stops new chunks
// from starting; runChunked then throws an AbortError so callers can
// tell a cancel apart from a failure.
const CONTENT_CONCURRENCY = 4;

function currentPageSlug() {
  try {
    return typeof window !== "undefined" ? window.location.pathname : "";
  } catch {
    return "";
  }
}

async function runChunked(texts, runChunk, onProgress, options = {}) {
  const { signal } = options;
  const chunks = groupTexts(texts);
  const results = new Array(texts.length).fill(null);
  let processed = 0;
  const reportProgress = () => {
    if (onProgress) onProgress(processed, texts.length);
  };

  // Start offset of each chunk inside `texts`, so parallel chunks can
  // still write their results into the right slots.
  const chunkStarts = [];
  let chunkStart = 0;
  for (const chunk of chunks) {
    chunkStarts.push(chunkStart);
    chunkStart += chunk.length;
  }

  const { aborted } = await runWithConcurrency(
    chunks,
    CONTENT_CONCURRENCY,
    async (chunk, chunkIdx) => {
      const start = chunkStarts[chunkIdx];
      let chunkDone = false;
      try {
        const translated = await runChunk(chunk, { signal });
        if (Array.isArray(translated) && translated.length === chunk.length) {
          chunk.forEach((_, i) => {
            results[start + i] = translated[i];
          });
          chunkDone = true;
        }
        // On a count mismatch we fall through to the per-element retry
        // below instead of guessing which translation belongs where.
      } catch (err) {
        if (err?.name === "AbortError") throw err;
        console.error("AI content request failed for a chunk:", err);
      }
      if (!chunkDone) {
        // Fallback: rewrite each element on its own. A single text has no
        // delimiter for the model to lose, so this succeeds even when the
        // batched reply comes back merged into one passage (which is what
        // used to break the "Intermediate" level entirely). Sequential
        // inside this chunk only — the other chunk workers keep running.
        for (let i = 0; i < chunk.length; i++) {
          if (signal?.aborted) {
            const abortError = new Error("Content request cancelled");
            abortError.name = "AbortError";
            throw abortError;
          }
          try {
            const single = await runChunk([chunk[i]], { signal });
            if (Array.isArray(single) && single.length === 1 && single[0]) {
              results[start + i] = single[0];
            }
          } catch (err) {
            if (err?.name === "AbortError") throw err;
            console.error("AI content request failed for an element:", err);
          }
          processed += 1;
          reportProgress();
        }
      } else {
        processed += chunk.length;
        reportProgress();
      }
    },
    {
      signal,
      onItemError: (err, chunkIdx) => {
        console.error("AI content request failed for a chunk:", err, chunkIdx);
      },
    }
  );

  if (aborted || signal?.aborted) {
    // Cancelled: throw so callers can tell a cancel from a failure —
    // but carry the finished chunks with the error. The caller applies
    // them, so work that is already done is not thrown away just
    // because the user stopped the rest.
    const abortError = new Error("Content request cancelled");
    abortError.name = "AbortError";
    abortError.partialResults = results;
    throw abortError;
  }
  return results;
}

/**
 * Translate many page texts to Urdu, a few elements per API call.
 * Returns an array aligned with `texts`; entries that could not be
 * translated are null (callers keep the original English text).
 */
export async function translateTexts(texts, onProgress, options = {}) {
  return runChunked(texts, async (chunk, { signal } = {}) => {
    const joined = chunk.join(`\n${CHUNK_DELIMITER}\n`);
    const response = await fetch(contentUrls.translate(), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text: joined, target_language: "Urdu", page: currentPageSlug() }),
      signal,
    });
    const full = await readSseStream(response);
    // Drop empty parts: the model sometimes echoes a trailing [[[BREAK]]]
    // even for a single passage. Without this, split() returns one extra
    // empty part, the length check in runChunked fails, and the whole chunk
    // falls back to per-element retries that hit the same echo.
    return full.split(CHUNK_DELIMITER).map((part) => part.trim()).filter((part) => part.length > 0);
  }, onProgress, options);
}

/**
 * Rewrite many page texts for a reading level ("beginner" or
 * "intermediate"). Same chunked approach as translateTexts; null entries
 * mean that element could not be rewritten and stays as written.
 */
export async function personalizeTexts(texts, level, onProgress, options = {}) {
  return runChunked(texts, async (chunk, { signal } = {}) => {
    const joined = chunk.join(`\n${CHUNK_DELIMITER}\n`);
    const response = await fetch(contentUrls.personalize(), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text: joined, level, page: currentPageSlug() }),
      signal,
    });
    const full = await readSseStream(response);
    // Drop empty parts: the model sometimes echoes a trailing [[[BREAK]]]
    // even for a single passage (see translateTexts above).
    return full.split(CHUNK_DELIMITER).map((part) => part.trim()).filter((part) => part.length > 0);
  }, onProgress, options);
}
