// Workstream C — Level rewrite speed: Neon-backed cache for AI rewrites.
//
// The `content_cache` table and the generic /api/cache endpoints already
// existed, but NOTHING used them: /api/translate and /api/personalize
// called OpenAI on every visit, so a long chapter paid the full rewrite
// cost again and again. This module is the single code path for that
// cache, used by the translate/personalize handlers and by /api/cache.
//
// Cache key: `v1:<kind>:<variant>:<page>:<sha256(text)>`
//   kind    "personalize" | "translate"
//   variant level ("beginner" | "intermediate") or target language ("Urdu")
//   page    chapter slug from the request (window.location.pathname),
//           sanitised; "unknown-page" when the caller does not send one.
//           The text hash already identifies the content, so the page is
//           a namespace for readability/invalidation, not for correctness.
//   text    the exact joined chunk the frontend sent (several passages
//           joined with [[[BREAK]]]); caching per chunk keeps each
//           Vercel invocation small and lets a repeat visit hit every
//           chunk instead of re-paying any OpenAI call.
//   v1      key version: bump when the system prompt or model changes so
//           stale rewrites are never served after a prompt change.
//
// The key is always under the 255-char content_cache PK (short prefixes
// + 64 hex chars). There is deliberately no TTL: the key is a pure
// function of the input text and the prompt version, so the same input
// always deserves the same output.
//
// Failure mode: the cache must never break a rewrite. Every helper
// catches DB errors and degrades to a miss / a skipped write, and each
// DB round-trip races a short timeout so a slow or dead database cannot
// stall the SSE stream.
import crypto from "node:crypto";

export const CONTENT_CACHE_KEY_VERSION = "v1";

// A cache lookup/write that is still pending after this long is treated
// as a miss / skipped write. Neon round-trips are normally tens of ms;
// 2s is far above that and far below any user-visible stall.
const CACHE_DB_TIMEOUT_MS = 2000;

export function normalizePageSlug(page) {
  if (typeof page !== "string" || !page.trim()) return "unknown-page";
  const slug = page
    .trim()
    .replace(/^https?:\/\/[^/]+/i, "")
    .split("?")[0]
    .split("#")[0]
    .replace(/[^a-z0-9/_-]+/gi, "-")
    .replace(/\/+/g, "/")
    .replace(/^\/|\/$/g, "")
    .slice(0, 80);
  return slug || "unknown-page";
}

export function contentCacheKey({ kind, variant, page, text }) {
  const hash = crypto
    .createHash("sha256")
    .update(typeof text === "string" ? text : "", "utf8")
    .digest("hex");
  const safeVariant = String(variant || "default")
    .replace(/[^a-z0-9-]+/gi, "-")
    .slice(0, 40);
  return `${CONTENT_CACHE_KEY_VERSION}:${kind}:${safeVariant}:${normalizePageSlug(page)}:${hash}`;
}

function withTimeout(promise, ms) {
  let timer;
  const timeout = new Promise((resolve) => {
    timer = setTimeout(() => resolve({ __cacheTimeout: true }), ms);
  });
  return Promise.race([promise, timeout]).finally(() => clearTimeout(timer));
}

export async function getCachedContent(pool, key, options = {}) {
  if (!pool || typeof key !== "string" || !key) return null;
  try {
    const result = await withTimeout(
      pool.query(`SELECT content FROM content_cache WHERE key = $1`, [key]),
      CACHE_DB_TIMEOUT_MS
    );
    if (!result || result.__cacheTimeout) return null;
    return result.rows.length > 0 ? result.rows[0].content : null;
  } catch (error) {
    if (options.throwOnError) throw error;
    console.error("Workstream C: content-cache read failed, treating as miss:", error.message);
    return null;
  }
}

export async function setCachedContent(pool, key, content, options = {}) {
  if (!pool || typeof key !== "string" || !key) return false;
  if (typeof content !== "string" || !content.trim()) return false;
  try {
    const result = await withTimeout(
      pool.query(
        `INSERT INTO content_cache (key, content) VALUES ($1, $2) ON CONFLICT (key) DO UPDATE SET content = EXCLUDED.content`,
        [key, content]
      ),
      CACHE_DB_TIMEOUT_MS
    );
    return Boolean(result && !result.__cacheTimeout);
  } catch (error) {
    if (options.throwOnError) throw error;
    console.error("Workstream C: content-cache write failed, continuing without cache:", error.message);
    return false;
  }
}
