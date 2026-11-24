/**
 * Shared language state for the reader controls (language menu +
 * Listen button). The translation itself lives in the language menu;
 * this module only holds the definitions, the current language, and
 * the chapter-element collection both controls read from.
 *
 * The current language is persisted on the reader's own device
 * (localStorage, "physical-ai-language"): a pick follows the reader
 * to the next chapter instead of resetting to English, and the
 * language menu re-applies it on the fresh page (the Neon cache
 * makes a visited page near-instant). Picking English clears the
 * stored choice. Nothing is re-translated silently for a reader who
 * never picked a language — the stored pick is the reader's own.
 */

// One event name for "stop reading now", shared by the language menu
// (a translation replaces the text being read) and the Level pill (a
// rewrite replaces it). The Listen button listens for it; the controls
// that fire it never import the Listen button.
export const CANCEL_SPEECH_EVENT = "physical-ai:cancel-speech";

// The seven languages the menu offers. `label` is the name the
// backend translates into (the /api/translate prompt interpolates it
// and the Neon cache keys on it, so each language caches separately
// and is shared across users). `speechLang` is the BCP-47 tag the
// Listen button asks the device's voices for. `fontClass` is the
// class applied to translated elements when the script needs its own
// face or direction; Latin-script languages need none (Inter).
export const LANGUAGES = [
  { code: "en", label: "English", nativeLabel: "English", speechLang: "en-US", dir: "ltr" },
  { code: "ur", label: "Urdu", nativeLabel: "اردو", speechLang: "ur-PK", dir: "rtl", fontClass: "translated-ur" },
  { code: "hi", label: "Hindi", nativeLabel: "हिन्दी", speechLang: "hi-IN", dir: "ltr", fontClass: "translated-hi" },
  { code: "fr", label: "French", nativeLabel: "Français", speechLang: "fr-FR", dir: "ltr" },
  { code: "es", label: "Spanish", nativeLabel: "Español", speechLang: "es-ES", dir: "ltr" },
  { code: "ar", label: "Arabic", nativeLabel: "العربية", speechLang: "ar-SA", dir: "rtl", fontClass: "translated-ar" },
  { code: "de", label: "German", nativeLabel: "Deutsch", speechLang: "de-DE", dir: "ltr" },
];

export function getLanguage(code) {
  return LANGUAGES.find((l) => l.code === code) || LANGUAGES[0];
}

// Tiny store for the current language: the menu writes it, the Listen
// button subscribes to it. No React context — the two controls are
// sibling navbar items rendered by Docusaurus, and a module store
// keeps them in sync without touching the theme's Root.
let currentLanguage = "en";
const listeners = new Set();

export function getCurrentLanguage() {
  return currentLanguage;
}

export function setCurrentLanguage(code) {
  const next = getLanguage(code).code;
  if (next === currentLanguage) return;
  currentLanguage = next;
  listeners.forEach((fn) => {
    try {
      fn(next);
    } catch {
      // One failing subscriber must not break the others.
    }
  });
}

export function subscribeLanguage(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

// The language the reader last picked, kept on their own device so
// it follows them across chapters. English is the default and is
// stored as "no stored pick" (the key is removed), so a reader who
// never picked a language always starts in English.
const LANGUAGE_STORAGE_KEY = "physical-ai-language";

export function readStoredLanguage() {
  try {
    const stored = localStorage.getItem(LANGUAGE_STORAGE_KEY);
    return LANGUAGES.some((l) => l.code === stored) ? stored : null;
  } catch {
    return null;
  }
}

export function storeLanguage(code) {
  try {
    if (code === "en") {
      localStorage.removeItem(LANGUAGE_STORAGE_KEY);
    } else {
      localStorage.setItem(LANGUAGE_STORAGE_KEY, code);
    }
  } catch {
    // Private mode etc. — the pick still applies to this page.
  }
}

import { elementToMarkdown } from "./markdown";

// The chapter elements a reader control works on: paragraphs,
// headings and list items inside the rendered Markdown, in document
// order. Code blocks are skipped (they are not prose to translate or
// to read aloud), and so are elements with almost no text.
//
// The set also covers h4–h6 (quiz question titles and deep section
// heads — the translation set already had them, the Listen set did
// not, so a translated chapter's quiz questions stayed silent) and
// <label> (a quiz option's text: the only labels on a doc page are
// quiz options). The question text span inside a quiz h4 is NOT
// collected separately — the h4 itself carries it, so collecting
// both would read the question twice.
export function collectChapterElements(minLength = 2) {
  if (typeof document === "undefined") return [];
  const found = document.querySelectorAll(
    ".markdown p, .markdown h1, .markdown h2, .markdown h3, .markdown h4, .markdown h5, .markdown h6, .markdown li, .markdown label"
  );
  const out = [];
  for (const element of Array.from(found)) {
    if (element.closest("pre") || element.closest("code")) continue;
    const text = (element.innerText || "").trim();
    if (text.length < minLength) continue;
    out.push({ element, text });
  }
  return out;
}

// Same chapter elements, but the text is the element's HTML converted
// to inline markdown (see elementToMarkdown) instead of plain
// innerText — the translator needs to SEE the **bold** markers to
// keep them. The Listen button keeps collectChapterElements: spoken
// text must stay plain, with no markers to read aloud.
//
// The set is wider than the Listen set on purpose: h4–h6 (quiz
// question titles and deep section heads) and table cells (the
// comparison tables) are prose a reader expects translated too —
// leaving them in English inside an otherwise translated chapter
// reads as a broken translation.
//
// One exception: a quiz question title (h4[data-quiz-question]) is
// NOT collected as a whole. It carries its number ("1.") and its
// difficulty badge as sibling spans of the question text, and the
// translator renders over the whole element (applyRewrittenText
// replaces innerHTML) — translating the h4 would wipe both. The
// question text itself lives in a [data-quiz-question-text] span
// inside the h4 (see Quiz/index.js); that span is collected instead,
// so the number and badge survive the translation untouched.
export function collectChapterMarkdown(minLength = 2) {
  if (typeof document === "undefined") return [];
  const found = document.querySelectorAll(
    ".markdown p, .markdown h1, .markdown h2, .markdown h3, .markdown h4, .markdown h5, .markdown h6, .markdown li, .markdown td, .markdown th, .markdown [data-quiz-question-text]"
  );
  const out = [];
  for (const element of Array.from(found)) {
    if (element.matches("h4[data-quiz-question]")) continue;
    if (element.closest("pre") || element.closest("code")) continue;
    const text = elementToMarkdown(element).trim();
    if (text.length < minLength) continue;
    out.push({ element, text });
  }
  return out;
}
