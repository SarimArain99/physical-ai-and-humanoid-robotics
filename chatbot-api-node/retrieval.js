/**
 * Workstream D — True RAG with Qdrant (retrieval module).
 *
 * The chat backend used to stuff the whole textbook (~434 KB) into the
 * system prompt of every session and call that RAG. It was not retrieval:
 * no embeddings, no vector search, no grounding, no sources.
 *
 * This module is the real thing:
 *   - chunkMarkdown(): splits one chapter file into retrievable chunks
 *     with chapter / week / heading / URL metadata (shared by the ingest
 *     script, so ingest and search agree on chunking).
 *   - embedText(): OpenAI text-embedding-3-small (1536 dims).
 *   - searchQdrant(): embeds the question, searches the `physical-ai-book`
 *     Qdrant collection top-k, returns sources with citations.
 *   - buildGroundedSystemPrompt(): the per-request system prompt built
 *     ONLY from the retrieved chunks (plus selected text, when given).
 *
 * Qdrant access is plain HTTPS (REST) via global fetch — no extra
 * dependency. QDRANT_URL is the cluster URL as shown in Qdrant Cloud
 * (e.g. https://<id>.<region>.aws.cloud.qdrant.io, no :6333 port).
 * Both QDRANT_URL and QDRANT_API_KEY come from process.env only.
 * Nothing here hardcodes a URL or a key, and nothing logs either value.
 */

import { createHash } from "node:crypto";

export const QDRANT_COLLECTION = "physical-ai-book";
export const EMBEDDING_MODEL = "text-embedding-3-small";
export const VECTOR_SIZE = 1536;
export const TOP_K = 5;

// Chunks target ~1200 characters with ~200 characters of overlap, cut at
// paragraph boundaries so a chunk never starts mid-sentence.
export const CHUNK_TARGET_CHARS = 1200;
export const CHUNK_OVERLAP_CHARS = 200;

/**
 * Qdrant configuration from the environment. `configured` is false until
 * BOTH vars are set; callers must fall back (never pretend grounded).
 * `missing` names exactly which vars are absent — for the log line and
 * /api/health — never their values.
 */
export function getRagConfig() {
  const url = (process.env.QDRANT_URL || "").trim().replace(/\/+$/, "");
  const apiKey = process.env.QDRANT_API_KEY || "";
  const missing = [];
  if (!url) missing.push("QDRANT_URL");
  if (!apiKey) missing.push("QDRANT_API_KEY");
  return {
    url,
    apiKey,
    configured: missing.length === 0,
    missing,
    collection: QDRANT_COLLECTION,
    embeddingModel: EMBEDDING_MODEL,
    vectorSize: VECTOR_SIZE,
    topK: TOP_K,
  };
}

/** One-line status for the startup log and /api/health. */
export function getRagStatus() {
  const cfg = getRagConfig();
  return cfg.configured
    ? { qdrant: "configured", collection: cfg.collection, topK: cfg.topK, embeddingModel: cfg.embeddingModel }
    : { qdrant: "missing", missing: cfg.missing, collection: cfg.collection, topK: cfg.topK, embeddingModel: cfg.embeddingModel };
}

function qdrantHeaders(apiKey) {
  return { "Content-Type": "application/json", "api-key": apiKey };
}

async function qdrantRequest(path, { method = "GET", body = undefined, fetchImpl = fetch } = {}) {
  const cfg = getRagConfig();
  if (!cfg.configured) {
    throw new Error(`Qdrant not configured (missing ${cfg.missing.join(", ")}).`);
  }
  const res = await fetchImpl(`${cfg.url}${path}`, {
    method,
    headers: qdrantHeaders(cfg.apiKey),
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  if (!res.ok) {
    const detail = await res.text().catch(() => "");
    throw new Error(`Qdrant ${method} ${path} failed (${res.status}): ${detail.slice(0, 200)}`);
  }
  return res.json();
}

/** Embed one text (or an array of texts) with text-embedding-3-small. */
export async function embedTexts(openai, texts) {
  const input = Array.isArray(texts) ? texts : [texts];
  const res = await openai.embeddings.create({ model: EMBEDDING_MODEL, input });
  return res.data.map((d) => d.embedding);
}

export async function embedText(openai, text) {
  const [embedding] = await embedTexts(openai, [text]);
  return embedding;
}

/**
 * Search the book for the question. Always resolves — never throws for a
 * missing/unreachable Qdrant — so the chat handler can fall back cleanly.
 * Returns { available, configured, sources, error } where sources are
 * [{ chapter, week, title, heading, url, text, score }].
 */
export async function searchQdrant(openai, query, { topK = TOP_K, fetchImpl = fetch } = {}) {
  const cfg = getRagConfig();
  if (!cfg.configured) {
    return { available: false, configured: false, sources: [], error: `Qdrant not configured (missing ${cfg.missing.join(", ")}).` };
  }
  try {
    const queryVector = await embedText(openai, query);
    const data = await qdrantRequest(`/collections/${QDRANT_COLLECTION}/points/search`, {
      method: "POST",
      body: { vector: queryVector, limit: topK, with_payload: true },
      fetchImpl,
    });
    const sources = (data.result || []).map((hit) => {
      const p = hit.payload || {};
      return {
        chapter: p.chapter || p.title || "Unknown chapter",
        week: typeof p.week === "number" ? p.week : null,
        title: p.title || p.chapter || "Unknown chapter",
        heading: p.heading || "",
        url: p.url || "",
        text: p.text || "",
        score: typeof hit.score === "number" ? hit.score : null,
      };
    });
    return { available: true, configured: true, sources, error: null };
  } catch (err) {
    console.error("[RAG] Qdrant search failed, falling back to ungrounded chat:", err.message);
    return { available: false, configured: true, sources: [], error: err.message };
  }
}

/** Short system prompt used when retrieval is unavailable (fallback). */
export const FALLBACK_SYSTEM_PROMPT = `You are the Physical AI & Humanoid Robotics Textbook Assistant, helping students learn about ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) for humanoid robots. Answer in a very easy, simple, beginner-friendly way. Only answer questions about the Physical AI & Humanoid Robotics course; for anything outside it, politely refuse and redirect the student back to the course topics. If you are unsure of an answer, say so instead of guessing.`;

/**
 * The grounded per-request system prompt: the model may ONLY use the
 * retrieved passages below (plus the selected text, when the student
 * highlighted one). Every factual claim must be traceable to a passage,
 * and the answer must cite the chapter and week it came from.
 */
export function buildGroundedSystemPrompt(sources, { selectedText = "" } = {}) {
  const passages = sources
    .map((s, i) => {
      const cite = s.week ? `Week ${s.week}` : "Introduction";
      return `[Source ${i + 1} — ${s.title} (${cite})]\n${s.text}`;
    })
    .join("\n\n");
  const selected = selectedText
    ? `\n\nThe student highlighted this text from the book — it is the primary context for their question:\n"${selectedText}"\n`
    : "";
  return `You are the Physical AI & Humanoid Robotics Textbook Assistant, helping students learn about ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) for humanoid robots. Answer in a very easy, simple, beginner-friendly way.
${selected}
Answer ONLY from the retrieved textbook passages below. Do not use outside knowledge. If the passages do not contain the answer, say plainly that the book does not cover it here and suggest the closest related chapter — do not guess.
After answering, name the chapter and week each key point came from (for example: "Week 3, ROS 2 Architecture and Core Concepts").

Retrieved passages:
${passages}`;
}

// ---------------------------------------------------------------------------
// Chunking (shared by the ingest script and the tests)
// ---------------------------------------------------------------------------

function stripFrontmatter(text) {
  return text.replace(/^---\n[\s\S]*?\n---\n/, "");
}

function cleanMarkdown(text) {
  return stripFrontmatter(text)
    .split("\n")
    .filter((line) => !line.trim().startsWith("import "))
    .join("\n")
    .replace(/```[a-zA-Z]*\n/g, "")
    .replace(/```/g, "")
    .replace(/\n{3,}/g, "\n\n")
    .trim();
}

/**
 * Metadata for one chapter file. `relPath` is relative to frontend/docs/
 * (e.g. "week3/ros2-nodes-topics.md" or "intro.md"). The URL is the
 * Docusaurus route: /docs/<relPath without .md>.
 */
export function chapterMeta(relPath, content) {
  const clean = cleanMarkdown(content);
  const titleMatch = clean.match(/^#\s+(.+)$/m);
  const title = titleMatch ? titleMatch[1].trim() : relPath.replace(/\.mdx?$/, "");
  const weekMatch = relPath.match(/^week(\d+)\//);
  const week = weekMatch ? Number(weekMatch[1]) : 0;
  const chapter = week > 0 ? `Week ${week}` : "Introduction";
  const url = `/docs/${relPath.replace(/\.mdx?$/, "")}`;
  return { chapter, week, title, url, sourceFile: relPath };
}

/**
 * Split one chapter into chunks of ~1200 chars with ~200 chars overlap,
 * cut at paragraph boundaries, never mid-sentence. Each chunk keeps its
 * section heading so a retrieved chunk is self-describing. Returns
 * [{ text, heading, chunkIndex, ...meta }]; `embedText` for a chunk is
 * heading + text (the stored payload text stays clean).
 */
export function chunkMarkdown(relPath, content) {
  const meta = chapterMeta(relPath, content);
  const clean = cleanMarkdown(content);
  // Split into sections at ## / ### headings; the H1 title is not a chunk.
  const sections = [];
  let current = { heading: "", lines: [] };
  for (const line of clean.split("\n")) {
    const heading = line.match(/^#{2,3}\s+(.+)$/);
    if (heading) {
      if (current.lines.join("\n").trim()) sections.push(current);
      current = { heading: heading[1].trim(), lines: [] };
    } else if (line.startsWith("# ")) {
      // Chapter title — metadata only, not chunk content.
    } else {
      current.lines.push(line);
    }
  }
  if (current.lines.join("\n").trim()) sections.push(current);

  const chunks = [];
  for (const section of sections) {
    const paragraphs = section.lines
      .join("\n")
      .split(/\n\s*\n/)
      .map((p) => p.replace(/\s*\n\s*/g, " ").trim())
      .filter(Boolean);
    let currentChunk = "";
    const flush = () => {
      if (!currentChunk.trim()) return;
      chunks.push({ ...meta, heading: section.heading, text: currentChunk.trim(), chunkIndex: chunks.length });
      // Overlap: carry the tail of this chunk into the next one.
      const tail = currentChunk.slice(-CHUNK_OVERLAP_CHARS);
      const cut = tail.search(/\s/);
      currentChunk = cut >= 0 ? tail.slice(cut + 1) : tail;
    };
    for (const para of paragraphs) {
      if (currentChunk.length + para.length + 1 > CHUNK_TARGET_CHARS && currentChunk.trim()) {
        flush();
      }
      currentChunk = currentChunk ? `${currentChunk} ${para}` : para;
    }
    flush();
    // Do not carry overlap across section boundaries.
    currentChunk = "";
    // The flush above already pushed the last chunk of this section; the
    // overlap tail it left in currentChunk is discarded by the reset.
  }
  return chunks.filter((c) => c.text.length > 0);
}

/** Text embedded for a chunk: heading context + clean chunk text. */
export function chunkEmbedText(chunk) {
  return chunk.heading ? `${chunk.heading}\n${chunk.text}` : chunk.text;
}

/** Deterministic Qdrant point id (UUID form of the md5 of file:index). */
export function chunkPointId(sourceFile, chunkIndex) {
  const hex = createHash("md5").update(`${sourceFile}:${chunkIndex}`).digest("hex");
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20, 32)}`;
}

// Re-exported for the ingest script.
export { qdrantRequest };
