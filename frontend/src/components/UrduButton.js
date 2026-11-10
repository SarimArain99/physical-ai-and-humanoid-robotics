/**
 * Urdu Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Urdu/English language toggle with unified styling.
 */
import React, { useState, useRef } from "react";
import { useAuth } from "./Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { translateTexts } from "../config/api";
import "./UrduButton.css";

const GlobeIcon = () => (
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
    <circle cx="12" cy="12" r="10"></circle>
    <line x1="2" y1="12" x2="22" y2="12"></line>
    <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"></path>
  </svg>
);

const UrduButtonContent = () => {
  const { user } = useAuth();
  const [isUrdu, setIsUrdu] = useState(false);
  const [loading, setLoading] = useState(false);
  // Workstream C: honest counts (texts translated of total), same as the
  // Level pill — and the same cancel behaviour: tapping while running
  // stops every in-flight chunk request.
  const [progress, setProgress] = useState({ done: 0, total: 0 });
  const abortRef = useRef(null);

  // Hide button if user is not logged in
  if (!user) return null;

  const handleTranslate = async () => {
    // Workstream C: while a translation is running, the button cancels it.
    if (loading) {
      abortRef.current?.abort();
      return;
    }

    // If already translated, reload to reset to English
    if (isUrdu) {
      window.location.reload();
      return;
    }

    setLoading(true);
    setProgress({ done: 0, total: 0 });

    const controller = new AbortController();
    abortRef.current = controller;

    // Declared here (not inside try) so the AbortError branch below can
    // apply the finished chunks too.
    const validElements = [];
    const textsToTranslate = [];
    const applyTranslations = (list) => {
      let count = 0;
      (list || []).forEach((translation, idx) => {
        const element = validElements[idx];
        if (translation && element) {
          count += 1;
          element.innerText = translation;
          // Nastaliq face and roomier leading only: the text keeps
          // the page's own colour and alignment in both themes.
          element.classList.add("urdu-translated");
        }
      });
      return count;
    };

    try {
      // Select all text content in the documentation
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown h1, .markdown h2, .markdown h3, .markdown li"
      );

      if (contentElements.length === 0) {
        alert("No content found to translate.");
        setLoading(false);
        return;
      }

      const elementsArray = Array.from(contentElements);

      // Filter valid elements and collect texts for batching
      for (const element of elementsArray) {
        const originalText = element.innerText;
        // Skip very short text or code blocks
        if (originalText.trim().length < 2) continue;
        if (element.closest("pre") || element.closest("code")) continue;

        validElements.push(element);
        textsToTranslate.push(originalText);
      }

      const total = textsToTranslate.length;
      if (total === 0) {
        alert("No translatable content found.");
        setLoading(false);
        return;
      }

      // Translate a few elements per API call (the backend translates one
      // text per call and rate-limits AI requests). Chunks run concurrently
      // with a bound of 4 and cached chunks come back from Neon (Workstream
      // C), so a repeat visit is near-instant. Elements that could not be
      // translated keep their original English text.
      const translations = await translateTexts(
        textsToTranslate,
        (done, totalCount) => setProgress({ done, total: totalCount }),
        { signal: controller.signal }
      );

      const translatedCount = applyTranslations(translations);

      if (translatedCount === 0) {
        alert("Translation failed. The translation service is unavailable right now — please try again later.");
        setLoading(false);
        setProgress({ done: 0, total: 0 });
        return;
      }

      setIsUrdu(true);
    } catch (error) {
      // Workstream C: a cancel is not a failure — apply the chunks that
      // already finished (runChunked carries them on the error as
      // partialResults) and say nothing. If anything was translated,
      // the page is (partly) Urdu, so the toggle reads EN.
      if (error?.name === "AbortError") {
        if (applyTranslations(error.partialResults) > 0) setIsUrdu(true);
        return;
      }
      console.error(error);
      alert("Translation failed. The translation service is unavailable right now — please try again later.");
    } finally {
      abortRef.current = null;
      setLoading(false);
      setProgress({ done: 0, total: 0 });
    }
  };

  const getLabel = () => {
    if (loading) return progress.total > 0 ? `${progress.done} of ${progress.total}` : "Generating…";
    if (isUrdu) return "EN";
    return "اردو"; // Urdu in Urdu script
  };

  return (
    <button
      className={`navbar-btn navbar-btn--secondary urdu-button ${loading ? "navbar-btn--loading" : ""}`}
      onClick={handleTranslate}
      title={loading ? "Translating — tap to cancel" : isUrdu ? "Switch to English" : "Switch to Urdu translation"}
      aria-label={loading ? "Cancel translation" : isUrdu ? "Switch to English" : "Switch to Urdu"}
    >
      <span className="urdu-icon"><GlobeIcon /></span>
      <span className="navbar-btn__label">{getLabel()}</span>
    </button>
  );
};

const UrduButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <UrduButtonContent />}</BrowserOnly>
  );
};

export default UrduButton;
