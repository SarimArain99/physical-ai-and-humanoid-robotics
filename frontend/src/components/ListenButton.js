/**
 * Listen Button
 *
 * Reads the current chapter aloud with the voices already on the
 * reader's device (Web Speech API — no server, no cost, no audio
 * files). The voice matches the language the reader picked in the
 * language menu, or English when nothing is translated.
 *
 * Pause is implemented as cancel-and-remember-the-place, not the
 * native pause/resume: on Android Chrome the native pair is
 * unreliable (a paused utterance often never resumes). Cancelling
 * and re-speaking from the saved chunk is the same experience and
 * works everywhere. Long chapters are split into sentence-sized
 * utterances, so pause and stop answer at once instead of waiting
 * out a minutes-long utterance, and the sentence being read is
 * highlighted on the page.
 *
 * Honest states: when the device has no voice for the language, the
 * button says "No voice" and explains why instead of reading in the
 * wrong language. Reading stops on navigation, on a language or
 * level change, when the reader taps stop, and when the chapter
 * ends. Only chapters have anything to read, so the button hides
 * off the docs pages.
 */
import React, { useState, useRef, useEffect, useCallback } from "react";
import { useLocation } from "@docusaurus/router";
import BrowserOnly from "@docusaurus/BrowserOnly";
import {
  getLanguage,
  getCurrentLanguage,
  subscribeLanguage,
  collectChapterElements,
  CANCEL_SPEECH_EVENT,
} from "../utils/language";
import { pickVoice, splitIntoSentences } from "../utils/speech";
import "./ListenButton.css";

const SpeakerIcon = () => (
  <svg
    width="18"
    height="18"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"></polygon>
    <path d="M15.54 8.46a5 5 0 0 1 0 7.07"></path>
    <path d="M19.07 4.93a10 10 0 0 1 0 14.14"></path>
  </svg>
);

const PlayIcon = () => (
  <svg
    width="14"
    height="14"
    viewBox="0 0 24 24"
    fill="currentColor"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <polygon points="6 3 20 12 6 21 6 3"></polygon>
  </svg>
);

const PauseIcon = () => (
  <svg
    width="14"
    height="14"
    viewBox="0 0 24 24"
    fill="currentColor"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <rect x="6" y="4" width="4" height="16"></rect>
    <rect x="14" y="4" width="4" height="16"></rect>
  </svg>
);

const StopIcon = () => (
  <svg
    width="12"
    height="12"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2.5"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <line x1="18" y1="6" x2="6" y2="18"></line>
    <line x1="6" y1="6" x2="18" y2="18"></line>
  </svg>
);

function speechSupported() {
  return (
    typeof window !== "undefined" &&
    "speechSynthesis" in window &&
    typeof window.speechSynthesis !== "undefined"
  );
}

const ListenButtonContent = ({ onClick: onNavbarItemClick }) => {
  const location = useLocation();
  const isDocsPage = location.pathname.startsWith("/docs/");
  // idle | playing | paused | novoice (no voice for this language)
  const [status, setStatus] = useState("idle");
  const [language, setLanguage] = useState(() => getCurrentLanguage());
  const [voices, setVoices] = useState([]);
  const [progress, setProgress] = useState({ index: 0, total: 0 });
  // The utterances for this reading: { element, text } per sentence.
  const chunksRef = useRef([]);
  // Index of the chunk being read (or to resume from).
  const indexRef = useRef(0);
  // Monotonic session: a cancelled session's late onend handlers must
  // not start the next chunk.
  const sessionRef = useRef(0);
  const statusRef = useRef("idle");
  useEffect(() => {
    statusRef.current = status;
  }, [status]);

  // Voices arrive asynchronously (getVoices is often empty on the
  // first call); refresh when the browser says they changed.
  useEffect(() => {
    if (!speechSupported()) return;
    const load = () => setVoices(window.speechSynthesis.getVoices());
    load();
    window.speechSynthesis.addEventListener("voiceschanged", load);
    return () =>
      window.speechSynthesis.removeEventListener("voiceschanged", load);
  }, []);

  // A language change stops any reading: the words under it are
  // about to be replaced by another language's text.
  useEffect(() => subscribeLanguage((code) => {
    stopReading();
    setLanguage(code);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }), []);

  // Navigation ends a reading; the old chapter is gone.
  useEffect(() => {
    stopReading();
    setProgress({ index: 0, total: 0 });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [location.pathname]);

  // A level rewrite or a translation replaces the text being read.
  useEffect(() => {
    const onCancel = () => stopReading();
    window.addEventListener(CANCEL_SPEECH_EVENT, onCancel);
    return () => window.removeEventListener(CANCEL_SPEECH_EVENT, onCancel);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Unmount: never leave the device talking after the button is gone.
  useEffect(() => () => {
    try {
      if (speechSupported()) window.speechSynthesis.cancel();
    } catch {
      // Ignore teardown errors.
    }
  }, []);

  const clearHighlight = useCallback(() => {
    for (const chunk of chunksRef.current) {
      chunk.element.classList?.remove("listen-reading");
    }
  }, []);

  const highlight = useCallback((chunk) => {
    clearHighlight();
    if (!chunk) return;
    chunk.element.classList?.add("listen-reading");
    // Keep the sentence being read on screen, but never yank the
    // page while the reader is looking elsewhere.
    try {
      const rect = chunk.element.getBoundingClientRect();
      if (rect.top < 80 || rect.bottom > window.innerHeight - 80) {
        chunk.element.scrollIntoView({ behavior: "smooth", block: "center" });
      }
    } catch {
      // A detached element has no rect; reading continues regardless.
    }
  }, [clearHighlight]);

  const stopReading = useCallback(() => {
    sessionRef.current += 1;
    indexRef.current = 0;
    try {
      if (speechSupported()) window.speechSynthesis.cancel();
    } catch {
      // Ignore teardown errors.
    }
    clearHighlight();
    if (statusRef.current !== "idle") setStatus("idle");
    setProgress({ index: 0, total: 0 });
  }, [clearHighlight]);

  const speakChunk = useCallback((index, session) => {
    const chunks = chunksRef.current;
    if (session !== sessionRef.current) return;
    if (index >= chunks.length) {
      // The chapter ended.
      setStatus("idle");
      setProgress({ index: 0, total: 0 });
      clearHighlight();
      return;
    }
    const chunk = chunks[index];
    indexRef.current = index;
    setProgress({ index: index + 1, total: chunks.length });
    highlight(chunk);
    const lang = getLanguage(language);
    const utterance = new SpeechSynthesisUtterance(chunk.text);
    utterance.lang = lang.speechLang;
    const voice = pickVoice(voices, lang.speechLang);
    if (voice) {
      // A voice that is not a genuine SpeechSynthesisVoice (a test
      // double, or a device voice that went stale) throws here — and
      // the throw used to stall the whole reading on "Playing" with
      // nothing spoken. The language tag alone still reads in the
      // right language, so a bad voice never blocks the chapter.
      try {
        utterance.voice = voice;
      } catch {
        // Ignore: the utterance reads with the default voice.
      }
    }
    utterance.onend = () => {
      if (session !== sessionRef.current) return;
      if (statusRef.current !== "playing") return;
      speakChunk(index + 1, session);
    };
    utterance.onerror = (event) => {
      // "canceled" is our own stop/pause — not an error.
      if (event?.error === "canceled" || event?.error === "interrupted") return;
      if (session !== sessionRef.current) return;
      console.error("Speech failed:", event?.error);
      setStatus("idle");
      clearHighlight();
    };
    window.speechSynthesis.speak(utterance);
  }, [clearHighlight, highlight, language, voices]);

  // Chrome drops an utterance spoken in the same tick as cancel(),
  // so a (re)start that follows a cancel waits a beat first.
  const startReading = useCallback((fromIndex, afterCancel) => {
    const lang = getLanguage(language);
    const voice = pickVoice(voices, lang.speechLang);
    if (!voice) {
      setStatus("novoice");
      alert(
        `No ${lang.label} voice is installed on this device, so the chapter cannot be read aloud in ${lang.label}. On Android, add one under Settings → Text-to-speech output, then try again.`
      );
      return;
    }
    const session = ++sessionRef.current;
    setStatus("playing");
    const run = () => speakChunk(fromIndex, session);
    if (afterCancel) {
      window.setTimeout(run, 100);
    } else {
      run();
    }
  }, [language, speakChunk, voices]);

  const handleToggle = () => {
    if (!speechSupported()) return;
    if (status === "playing") {
      // Pause = cancel and remember the place; resume re-speaks
      // from the saved chunk (see the file header for why).
      sessionRef.current += 1;
      try {
        window.speechSynthesis.cancel();
      } catch {
        // Ignore teardown errors.
      }
      setStatus("paused");
      return;
    }
    if (status === "paused") {
      startReading(indexRef.current, true);
      return;
    }
    if (status === "novoice") {
      // Re-check: a voice may have arrived since the last attempt.
      startReading(0, false);
      return;
    }
    // Idle: build the reading from the page as it reads right now —
    // translated text included, because the translation already
    // replaced the words in the DOM.
    if (onNavbarItemClick) onNavbarItemClick();
    const collected = collectChapterElements(2);
    if (collected.length === 0) {
      alert("Nothing to read on this page.");
      return;
    }
    const chunks = [];
    for (const { element, text } of collected) {
      for (const sentence of splitIntoSentences(text)) {
        chunks.push({ element, text: sentence });
      }
    }
    if (chunks.length === 0) {
      alert("Nothing to read on this page.");
      return;
    }
    chunksRef.current = chunks;
    indexRef.current = 0;
    startReading(0, false);
  };

  if (!isDocsPage || !speechSupported()) return null;

  const lang = getLanguage(language);
  const label =
    status === "playing"
      ? progress.total > 0
        ? `Playing ${progress.index} of ${progress.total}`
        : "Playing…"
      : status === "paused"
        ? "Paused"
        : status === "novoice"
          ? "No voice"
          : "Listen";
  const title =
    status === "playing"
      ? "Reading — tap to pause"
      : status === "paused"
        ? "Paused — tap to resume"
        : status === "novoice"
          ? `No ${lang.label} voice on this device — tap for details`
          : `Listen to this chapter in ${lang.label}`;

  return (
    <div className="listen-controls">
      <button
        className="navbar-btn navbar-btn--secondary listen-trigger"
        onClick={handleToggle}
        aria-label={title}
        title={title}
      >
        <span className="listen-icon"><SpeakerIcon /></span>
        <span className="navbar-btn__label">{label}</span>
        <span className="listen-state-icon">
          {status === "playing" ? <PauseIcon /> : status === "paused" ? <PlayIcon /> : null}
        </span>
      </button>
      {(status === "playing" || status === "paused") && (
        <button
          className="navbar-btn navbar-btn--secondary listen-stop"
          onClick={stopReading}
          aria-label="Stop listening"
          title="Stop listening"
        >
          <StopIcon />
        </button>
      )}
    </div>
  );
};

const ListenButton = (props) => {
  return (
    <BrowserOnly fallback={null}>
      {() => <ListenButtonContent {...props} />}
    </BrowserOnly>
  );
};

export default ListenButton;
