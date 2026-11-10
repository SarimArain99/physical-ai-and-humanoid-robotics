/**
 * Shared language state for the reader controls (language menu +
 * Listen button). The translation itself lives in the language menu;
 * this module only holds the definitions, the current language, and
 * the chapter-element collection both controls read from.
 *
 * The current language is per page, not persisted: a translation
 * applies to the page the reader is on, and navigating to a new
 * chapter starts in English again (the Neon cache makes re-picking
 * a language on a visited page near-instant, and nothing is
 * re-translated silently behind the reader's back).
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

// The chapter elements a reader control works on: paragraphs,
// headings and list items inside the rendered Markdown, in document
// order. Code blocks are skipped (they are not prose to translate or
// to read aloud), and so are elements with almost no text.
export function collectChapterElements(minLength = 2) {
  if (typeof document === "undefined") return [];
  const found = document.querySelectorAll(
    ".markdown p, .markdown h1, .markdown h2, .markdown h3, .markdown li"
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
