// Workstream C — Level rewrite speed: Neon-backed cache for AI rewrites.
//
// The `content_cache` table and the generic /api/cache endpoints already
// existed, but NOTHING used them: /api/translate and /api/personalize
// called OpenAI on every visit, so a long chapter paid the full rewrite
// cost again and again. This module is the single code path for that
// cache, used by the translate/personalize handlers and by /api/cache.
//
// Cache key: `<version>:<kind>:<variant>:<page>:<sha256(text)>`
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
//   version key version PER KIND: bump one kind's version when its
//           system prompt or model changes, so that kind's stale
//           entries are abandoned without throwing away the other
//           kind's cache. Translate went v1 -> v2 on 2026-11-10 (the
//           prompt now demands kept **bold** markers and, for Hindi,
//           Devanagari-only output); personalize stays v1, so cached
//           Level rewrites keep working. Old entries are never deleted
//           — they simply stop matching any lookup key.
//
// The key is always under the 255-char content_cache PK (short prefixes
// + 64 hex chars).
//
// TTL (2026-11-24 security fix): entries older than CACHE_TTL_MS are
// treated as a miss on read, so a poisoned or stale entry cannot live
// forever. The table carries created_at for this; the upsert refreshes
// it on every rewrite, so a live entry's age is the age of its newest
// write. The row itself is deleted lazily (best effort) when a stale
// entry is found — the read never waits on the delete.
//
// Failure mode: the cache must never break a rewrite. Every helper
// catches DB errors and degrades to a miss / a skipped write, and each
// DB round-trip races a short timeout so a slow or dead database cannot
// stall the SSE stream.
import crypto from "node:crypto";

// Key version per kind — see the header comment. Unknown kinds fall
// back to v1 so a typo can never silently abandon the whole cache.
export const CONTENT_CACHE_KEY_VERSIONS = {
  translate: "v2",
  personalize: "v1",
};

export function cacheKeyVersion(kind) {
  return CONTENT_CACHE_KEY_VERSIONS[kind] || "v1";
}

// Entries older than this are treated as a miss (see the header
// comment). 30 days is far longer than any chapter rewrite cycle, so a
// legitimate cached translation is never thrown away mid-use, while a
// poisoned entry has a bounded lifetime even if nobody invalidates it.
export const CACHE_TTL_MS = 30 * 24 * 60 * 60 * 1000;

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
  return `${cacheKeyVersion(kind)}:${kind}:${safeVariant}:${normalizePageSlug(page)}:${hash}`;
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
      pool.query(`SELECT content, created_at FROM content_cache WHERE key = $1`, [key]),
      CACHE_DB_TIMEOUT_MS
    );
    if (!result || result.__cacheTimeout) return null;
    if (result.rows.length === 0) return null;
    const row = result.rows[0];
    // TTL: a stale entry is a miss. Delete it lazily (best effort, never
    // awaited) so the table does not keep dead rows forever.
    if (row.created_at) {
      const ageMs = Date.now() - new Date(row.created_at).getTime();
      if (ageMs > CACHE_TTL_MS) {
        deleteCachedContent(pool, key).catch(() => {});
        return null;
      }
    }
    return row.content;
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
        `INSERT INTO content_cache (key, content, created_at) VALUES ($1, $2, CURRENT_TIMESTAMP)
         ON CONFLICT (key) DO UPDATE SET content = EXCLUDED.content, created_at = CURRENT_TIMESTAMP`,
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

// Delete one cached entry (admin-only invalidate endpoint). Returns true
// when a row was deleted. A missing table/row is not an error.
export async function deleteCachedContent(pool, key, options = {}) {
  if (!pool || typeof key !== "string" || !key) return false;
  try {
    const result = await withTimeout(
      pool.query(`DELETE FROM content_cache WHERE key = $1`, [key]),
      CACHE_DB_TIMEOUT_MS
    );
    return Boolean(result && !result.__cacheTimeout && result.rowCount > 0);
  } catch (error) {
    if (options.throwOnError) throw error;
    console.error("Workstream C: content-cache delete failed:", error.message);
    return false;
  }
}
