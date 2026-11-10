/**
 * Pure helpers for the Listen button (browser voices). Kept separate
 * from the component so the voice matching and the sentence splitting
 * can be unit-tested in Node — speechSynthesis itself only exists in
 * the browser, and on a device with no voices at all.
 */

// Pick the best voice for a language tag (e.g. "ur-PK") from the list
// the device reports. Prefer an exact match, then any voice whose tag
// starts with the same language ("ur" matches "ur-PK" and "ur-IN"),
// then — only as a last resort — a voice for the same base language
// reported with an underscore ("ur_PK"). A local (on-device) voice
// wins over a network voice at the same match level: it reads
// without a round trip, and it still works when the network drops.
// Returns null when the device has no voice for the language at all —
// the button says so honestly instead of reading in the wrong
// language.
export function pickVoice(voices, speechLang) {
  if (!Array.isArray(voices) || voices.length === 0) return null;
  const wanted = String(speechLang || "").toLowerCase().replace("_", "-");
  const base = wanted.split("-")[0];
  const score = (voice) => {
    const tag = String(voice.lang || "").toLowerCase().replace("_", "-");
    let match = -1;
    if (tag === wanted) match = 3;
    else if (tag.startsWith(`${base}-`)) match = 2;
    else if (tag === base) match = 1;
    if (match < 0) return -1;
    // Local voices first within a match level.
    return match * 2 + (voice.localService ? 1 : 0);
  };
  let best = null;
  let bestScore = -1;
  for (const voice of voices) {
    const s = score(voice);
    if (s > bestScore) {
      bestScore = s;
      best = voice;
    }
  }
  return best;
}

// Sentence terminators across the menu's languages: Latin full stop,
// question and exclamation marks, the Urdu/Arabic full stop (۔ U+06D4)
// and the Arabic question mark (؟ U+061F). A terminator only splits
// when followed by whitespace or the end of the text, so "e.g." and
// "v2.0" stay whole.
const SENTENCE_END = /([.!?۔؟])(\s+|$)/g;

// Split one element's text into speakable chunks of at most maxLen
// characters, cut at sentence ends. A single over-long sentence is
// cut at a comma or semicolon, and only as a last resort mid-phrase.
// Short chapters read element by element; long paragraphs become
// several utterances, so pause and stop answer promptly instead of
// waiting out a two-minute utterance.
// Abbreviations whose period must not end a sentence: dotted
// initials ("e.g.", "i.e.", "U.S.") and short titles.
const ABBREV_TITLES = new Set(["Mr", "Mrs", "Ms", "Dr", "St", "vs", "etc", "approx", "No"]);

function isAbbreviationPeriod(text, periodIndex) {
  const before = text.slice(0, periodIndex);
  const token = before.match(/([A-Za-z]+(?:\.[A-Za-z]+)*)$/)?.[1] || "";
  if (!token) return false;
  if (/^[A-Za-z](\.[A-Za-z])+$/.test(token)) return true;
  return ABBREV_TITLES.has(token);
}

export function splitIntoSentences(text, maxLen = 220) {
  const clean = String(text || "").replace(/\s+/g, " ").trim();
  if (!clean) return [];
  const sentences = [];
  let start = 0;
  SENTENCE_END.lastIndex = 0;
  let m;
  while ((m = SENTENCE_END.exec(clean)) !== null) {
    // A period that belongs to an abbreviation ("e.g.") is not a
    // sentence end — keep reading.
    if (m[1] === "." && isAbbreviationPeriod(clean, m.index)) continue;
    sentences.push(clean.slice(start, m.index + 1).trim());
    start = m.index + m[0].length;
  }
  if (start < clean.length) sentences.push(clean.slice(start).trim());
  const chunks = [];
  for (const sentence of sentences) {
    if (!sentence) continue;
    if (sentence.length <= maxLen) {
      chunks.push(sentence);
      continue;
    }
    // Over-long sentence: cut at commas/semicolons first.
    const parts = sentence.split(/(?<=[,;:])\s+/);
    let current = "";
    for (const part of parts) {
      if ((current + " " + part).trim().length <= maxLen) {
        current = (current + " " + part).trim();
        continue;
      }
      if (current) chunks.push(current);
      if (part.length <= maxLen) {
        current = part;
      } else {
        // Still too long: hard cut at maxLen on a word boundary.
        let rest = part;
        while (rest.length > maxLen) {
          let cut = rest.lastIndexOf(" ", maxLen);
          if (cut <= 0) cut = maxLen;
          chunks.push(rest.slice(0, cut).trim());
          rest = rest.slice(cut).trim();
        }
        current = rest;
      }
    }
    if (current) chunks.push(current);
  }
  return chunks.filter((c) => c.length > 0);
}
