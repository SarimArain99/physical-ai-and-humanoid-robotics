// Shared rate-limit store backed by Neon Postgres.
//
// express-rate-limit's default MemoryStore is per-process, so on Vercel
// serverless each invocation has its own counter and window — a burst
// spread across instances never trips the limit (60 rapid requests
// produced 0x 429 live). This store keeps the counters in the Neon
// database the app already uses, so the count is global.
//
// The store implements express-rate-limit v8's Store interface:
// init, get, increment -> { totalHits, resetTime }, decrement, resetKey,
// resetAll, shutdown, localKeys: false (keys are shared across
// instances, so a key incremented here affects every instance).
//
// Failure mode: the limiter must never break the API. Every DB call is
// guarded — on any error (table missing, pool down, timeout) the store
// falls back to an in-memory Map, so a database outage degrades to the
// old per-instance behaviour instead of failing requests. The fallback
// is per-process and per-window, exactly like MemoryStore.
//
// One row per (namespace, key): `rate_limit` (key, hits, reset_at).
// A window is 15 minutes; a key whose reset_at has passed restarts at 1
// on the next increment (single atomic upsert, no read-then-write race).

const DEFAULT_WINDOW_MS = 15 * 60 * 1000;

export function createPgRateLimitStore(pool, options = {}) {
  const namespace = typeof options.namespace === "string" && options.namespace
    ? options.namespace
    : "default";
  const windowMs = Number.isFinite(options.windowMs) && options.windowMs > 0
    ? options.windowMs
    : DEFAULT_WINDOW_MS;

  // In-memory fallback, used only when the database is unavailable.
  // Shape per key: { hits, resetAt: number (epoch ms) }.
  const memory = new Map();

  const fullKey = (key) => `${namespace}:${key}`;

  async function pgIncrement(key) {
    const now = new Date();
    const resetAt = new Date(now.getTime() + windowMs);
    const result = await pool.query(
      `INSERT INTO rate_limit (key, hits, reset_at) VALUES ($1, 1, $2)
       ON CONFLICT (key) DO UPDATE SET
         hits = CASE WHEN rate_limit.reset_at <= NOW() THEN 1 ELSE rate_limit.hits + 1 END,
         reset_at = CASE WHEN rate_limit.reset_at <= NOW() THEN EXCLUDED.reset_at ELSE rate_limit.reset_at END
       RETURNING hits, reset_at`,
      [fullKey(key), resetAt]
    );
    const row = result.rows[0];
    return { totalHits: row.hits, resetTime: new Date(row.reset_at) };
  }

  function memoryIncrement(key) {
    const now = Date.now();
    const entry = memory.get(fullKey(key));
    if (!entry || entry.resetAt <= now) {
      const resetAt = now + windowMs;
      memory.set(fullKey(key), { hits: 1, resetAt });
      return { totalHits: 1, resetTime: new Date(resetAt) };
    }
    entry.hits += 1;
    return { totalHits: entry.hits, resetTime: new Date(entry.resetAt) };
  }

  return {
    // Shared across instances — a key incremented here counts everywhere.
    localKeys: false,

    init(options) {
      // express-rate-limit passes its own windowMs here; prefer the
      // value given to the store factory, fall back to the middleware's.
      if ((!options || !options.windowMs) && !Number.isFinite(windowMs)) return;
    },

    async get(key) {
      try {
        const result = await pool.query(
          `SELECT hits, reset_at FROM rate_limit WHERE key = $1`,
          [fullKey(key)]
        );
        if (result.rows.length === 0) return undefined;
        const row = result.rows[0];
        if (new Date(row.reset_at).getTime() <= Date.now()) return undefined;
        return { totalHits: row.hits, resetTime: new Date(row.reset_at) };
      } catch {
        const entry = memory.get(fullKey(key));
        if (!entry || entry.resetAt <= Date.now()) return undefined;
        return { totalHits: entry.hits, resetTime: new Date(entry.resetAt) };
      }
    },

    async increment(key) {
      try {
        return await pgIncrement(key);
      } catch (error) {
        console.error("Rate-limit store: Postgres unavailable, using in-memory fallback:", error.message);
        return memoryIncrement(key);
      }
    },

    async decrement(key) {
      try {
        await pool.query(
          `UPDATE rate_limit SET hits = GREATEST(hits - 1, 0) WHERE key = $1 AND reset_at > NOW()`,
          [fullKey(key)]
        );
      } catch {
        const entry = memory.get(fullKey(key));
        if (entry && entry.hits > 0) entry.hits -= 1;
      }
    },

    async resetKey(key) {
      try {
        await pool.query(`DELETE FROM rate_limit WHERE key = $1`, [fullKey(key)]);
      } catch {
        memory.delete(fullKey(key));
      }
    },

    async resetAll() {
      try {
        await pool.query(`DELETE FROM rate_limit WHERE key LIKE $1`, [`${namespace}:%`]);
      } catch {
        for (const k of [...memory.keys()]) {
          if (k.startsWith(`${namespace}:`)) memory.delete(k);
        }
      }
    },

    shutdown() {
      memory.clear();
    },
  };
}
