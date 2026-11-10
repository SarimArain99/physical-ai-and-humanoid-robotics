#!/usr/bin/env node
/**
 * Workstream D — Ingest the Physical AI textbook into Qdrant.
 *
 * Reads the 13 week chapters (+ the Introduction) from frontend/docs/,
 * chunks them (see retrieval.js: ~1200 chars, ~200 overlap, cut at
 * paragraph boundaries, chapter + week + heading metadata), embeds each
 * chunk with OpenAI text-embedding-3-small (1536 dims) and upserts the
 * vectors into the Qdrant collection `physical-ai-book` (cosine).
 *
 * Idempotent: point ids are deterministic (md5 of file:chunkIndex), so
 * re-running replaces the same points instead of duplicating them.
 *
 * Run once the Qdrant Cloud cluster exists (the parent provisions it)
 * and QDRANT_URL + QDRANT_API_KEY are set in the backend's Vercel env:
 *
 *   OPENAI_API_KEY=<key> QDRANT_URL=https://<id>.<region>.aws.cloud.qdrant.io \
 *     QDRANT_API_KEY=<key> node scripts/ingest.js
 *
 * (From the chatbot-api-node/ directory. TEXTBOOK_DOCS_PATH overrides
 * the docs folder; --dry-run chunks and prints counts without calling
 * OpenAI or Qdrant.)
 *
 * No key is ever written to a file or printed — only presence is checked.
 */

import "dotenv/config";
import OpenAI from "openai";
import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";
import {
  QDRANT_COLLECTION,
  VECTOR_SIZE,
  EMBEDDING_MODEL,
  getRagConfig,
  getRagStatus,
  chunkMarkdown,
  chunkEmbedText,
  chunkPointId,
  embedTexts,
  qdrantRequest,
} from "../retrieval.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const DOCS_DIR = process.env.TEXTBOOK_DOCS_PATH || path.join(__dirname, "..", "..", "frontend", "docs");
const DRY_RUN = process.argv.includes("--dry-run");
const EMBED_BATCH = 32;
const UPSERT_BATCH = 100;

function fail(message) {
  console.error(`\n[ingest] ERROR: ${message}`);
  process.exit(1);
}

function listMarkdownFiles(dir, base = dir) {
  const out = [];
  for (const entry of fs.readdirSync(dir, { withFileTypes: true }).sort((a, b) => a.name.localeCompare(b.name))) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) out.push(...listMarkdownFiles(full, base));
    else if (entry.isFile() && /\.mdx?$/.test(entry.name)) out.push(path.relative(base, full));
  }
  return out;
}

async function ensureCollection() {
  try {
    await qdrantRequest(`/collections/${QDRANT_COLLECTION}`);
    console.log(`[ingest] Collection "${QDRANT_COLLECTION}" already exists — keeping it (upsert will replace points).`);
  } catch (err) {
    if (/404/.test(err.message)) {
      console.log(`[ingest] Creating collection "${QDRANT_COLLECTION}" (vectors: ${VECTOR_SIZE}, cosine)...`);
      await qdrantRequest(`/collections/${QDRANT_COLLECTION}`, {
        method: "PUT",
        body: { vectors: { size: VECTOR_SIZE, distance: "Cosine" } },
      });
      console.log(`[ingest] Collection created.`);
    } else {
      throw err;
    }
  }
}

async function main() {
  console.log("[ingest] Physical AI textbook → Qdrant ingest");
  console.log(`[ingest] Docs: ${DOCS_DIR}`);
  console.log(`[ingest] Collection: ${QDRANT_COLLECTION} | Embedding: ${EMBEDDING_MODEL} (${VECTOR_SIZE} dims, cosine)`);
  if (DRY_RUN) console.log("[ingest] --dry-run: chunking only, no OpenAI / Qdrant calls.");

  if (!process.env.OPENAI_API_KEY && !DRY_RUN) {
    fail("OPENAI_API_KEY is not set. Set it in the environment (same key the backend uses) and re-run.");
  }
  const rag = getRagConfig();
  if (!rag.configured && !DRY_RUN) {
    fail(`Qdrant is not configured (missing ${rag.missing.join(", ")}). Set QDRANT_URL and QDRANT_API_KEY in the backend's Vercel env vars (Qdrant Cloud cluster, account sarimarain99@gmail.com) and re-run. Nothing was ingested.`);
  }
  if (!fs.existsSync(DOCS_DIR)) fail(`Docs folder not found: ${DOCS_DIR}`);

  const files = listMarkdownFiles(DOCS_DIR);
  if (files.length === 0) fail(`No .md files found under ${DOCS_DIR}.`);
  console.log(`[ingest] Found ${files.length} chapter files.`);

  // Chunk everything first, so a bad file fails before any paid calls.
  const perChapter = [];
  let allChunks = [];
  for (const rel of files) {
    const content = fs.readFileSync(path.join(DOCS_DIR, rel), "utf-8");
    const chunks = chunkMarkdown(rel, content);
    perChapter.push({ file: rel, title: chunks[0]?.title || rel, week: chunks[0]?.week ?? null, count: chunks.length });
    allChunks = allChunks.concat(chunks);
  }
  console.log("\n[ingest] Chunks per chapter:");
  for (const row of perChapter) {
    const week = row.week > 0 ? `Week ${row.week}` : "Introduction";
    console.log(`  ${row.file} — ${row.title} (${week}): ${row.count} chunks`);
  }
  console.log(`[ingest] Total: ${allChunks.length} chunks from ${files.length} files.`);
  if (DRY_RUN) {
    console.log("[ingest] Dry run complete — no points written.");
    return;
  }

  await ensureCollection();

  const openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });

  // Embed in batches, then upsert in batches.
  const allEmbeddings = new Array(allChunks.length);
  for (let i = 0; i < allChunks.length; i += EMBED_BATCH) {
    const batch = allChunks.slice(i, i + EMBED_BATCH);
    const embeddings = await embedTexts(openai, batch.map(chunkEmbedText));
    embeddings.forEach((emb, j) => { allEmbeddings[i + j] = emb; });
    console.log(`[ingest] Embedded ${Math.min(i + EMBED_BATCH, allChunks.length)}/${allChunks.length} chunks...`);
  }

  const points = allChunks.map((chunk, i) => ({
    id: chunkPointId(chunk.sourceFile, chunk.chunkIndex),
    vector: allEmbeddings[i],
    payload: {
      chapter: chunk.chapter,
      week: chunk.week,
      title: chunk.title,
      heading: chunk.heading,
      url: chunk.url,
      text: chunk.text,
      chunk_index: chunk.chunkIndex,
      source_file: chunk.sourceFile,
    },
  }));
  for (let i = 0; i < points.length; i += UPSERT_BATCH) {
    await qdrantRequest(`/collections/${QDRANT_COLLECTION}/points`, {
      method: "PUT",
      body: { points: points.slice(i, i + UPSERT_BATCH) },
    });
    console.log(`[ingest] Upserted ${Math.min(i + UPSERT_BATCH, points.length)}/${points.length} points...`);
  }

  // Verify: count + a real search, so a silent empty collection is caught.
  const count = await qdrantRequest(`/collections/${QDRANT_COLLECTION}/points/count`, {
    method: "POST",
    body: {},
  });
  console.log(`\n[ingest] Collection now holds ${count.result?.count ?? "?"} points (expected ${points.length}).`);

  const probe = "What is ROS 2 and how do nodes and topics work?";
  console.log(`[ingest] Verification search: "${probe}"`);
  const probeVector = (await embedTexts(openai, [probe]))[0];
  const search = await qdrantRequest(`/collections/${QDRANT_COLLECTION}/points/search`, {
    method: "POST",
    body: { vector: probeVector, limit: 3, with_payload: true },
  });
  for (const hit of search.result || []) {
    const p = hit.payload || {};
    console.log(`  score ${Number(hit.score).toFixed(3)} — ${p.title} (Week ${p.week}) ${p.url} — "${(p.text || "").slice(0, 120)}..."`);
  }
  if (!search.result || search.result.length === 0) {
    fail("Verification search returned no results — the collection may be empty. Check the Qdrant Cloud collection and re-run.");
  }
  console.log("\n[ingest] Done. The chat API can now answer from these chapters (see /api/health for RAG status).");
}

main().catch((err) => {
  console.error(`\n[ingest] FAILED: ${err.message}`);
  process.exit(1);
});
