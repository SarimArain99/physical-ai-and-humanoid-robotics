/**
 * Language Menu
 *
 * Replaces the old Urdu-only toggle (the file keeps its name because
 * the snapshot push flow cannot carry a rename — the old file would
 * stay on GitHub as dead code). A globe button that opens a menu of
 * the book's languages: English (the original text) plus Urdu, Hindi,
 * French, Spanish, Arabic and German, translated on demand through
 * /api/translate and the shared Neon cache.
 *
 * The menu is always visible — signed out too. Translation needs a
 * signed-in account, so picking a language while signed out opens
 * the sign-in modal and the pick runs after sign-in. The menu works
 * in the desktop navbar and in the mobile hamburger drawer, where
 * it opens in flow below the trigger (an absolute menu would run
 * off the narrow drawer, the same clip the auth dropdown had).
 *
 * Translated text is rendered with the shared markdown renderer
 * (applyRewrittenText), not innerText — the old toggle wrote
 * innerText, so every "**" marker showed literally. The original
 * HTML of each translated element is kept in memory, so switching
 * back to English (or to another language) restores it without a
 * page reload. Urdu and Arabic read right-to-left in their own
 * faces; Hindi reads in Devanagari; the Latin languages keep Inter.
 */
import React, { useState, useRef, useEffect } from "react";
import { useLocation } from "@docusaurus/router";
import { useAuth } from "./Auth/AuthProvider";
import AuthModal from "./Auth/AuthModal";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { translateTexts } from "../config/api";
import { applyRewrittenText } from "../utils/markdown";
import {
  LANGUAGES,
  getLanguage,
  setCurrentLanguage,
  collectChapterElements,
  CANCEL_SPEECH_EVENT,
} from "../utils/language";
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

const ChevronDown = () => (
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
    <path d="M6 9l6 6 6-6" />
  </svg>
);

// Shown in place of the chevron while a translation is running: the
// trigger itself is the cancel control (tap it, or press Escape).
const CancelIcon = () => (
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

const CheckIcon = () => (
  <svg
    width="14"
    height="14"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2.5"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <polyline points="20 6 9 17 4 12"></polyline>
  </svg>
);

const LanguageMenuContent = ({ onClick: onNavbarItemClick }) => {
  const { user } = useAuth();
  const location = useLocation();
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [activeLanguage, setActiveLanguage] = useState("en");
  const [loading, setLoading] = useState(false);
  // Honest counts: texts actually translated out of the total on the
  // page — counts, never a percentage guessed from elapsed time.
  const [progress, setProgress] = useState({ done: 0, total: 0 });
  const dropdownRef = useRef(null);
  // The in-flight translation, so tapping the trigger or pressing
  // Escape while it runs cancels it instead of opening the menu.
  const abortRef = useRef(null);
  // Original HTML of every element this page's translation touched,
  // so switching back to English (or to another language) restores
  // the original text without a page reload.
  const originalsRef = useRef(new Map());
  // A pick made while signed out, run after sign-in completes.
  const pendingRef = useRef(null);
  // Monotonic run id: a cancelled run's late callbacks must not apply
  // over the run that replaced it.
  const runIdRef = useRef(0);

  // Close the menu on Escape or on a click outside it.
  useEffect(() => {
    if (!isOpen) return;
    const onKeyDown = (event) => {
      if (event.key === "Escape") setIsOpen(false);
    };
    const onPointerDown = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };
    document.addEventListener("keydown", onKeyDown);
    document.addEventListener("mousedown", onPointerDown);
    return () => {
      document.removeEventListener("keydown", onKeyDown);
      document.removeEventListener("mousedown", onPointerDown);
    };
  }, [isOpen]);

  // Escape while a translation is running cancels it.
  useEffect(() => {
    if (!loading) return;
    const onKeyDown = (event) => {
      if (event.key === "Escape") abortRef.current?.abort();
    };
    document.addEventListener("keydown", onKeyDown);
    return () => document.removeEventListener("keydown", onKeyDown);
  }, [loading]);

  // A new chapter starts in English: the translated elements of the
  // old page are gone with it, and the new page has never been
  // translated. Cancel anything in flight and reset the state.
  useEffect(() => {
    // Invalidate any in-flight run first, so its late callbacks
    // cannot apply over the fresh page or set state after the reset.
    runIdRef.current += 1;
    abortRef.current?.abort();
    originalsRef.current.clear();
    if (activeLanguage !== "en") {
      setActiveLanguage("en");
      setCurrentLanguage("en");
    }
    setIsOpen(false);
    // Only the pathname matters here; activeLanguage is read through
    // the state setter guard above, not as a dependency.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [location.pathname]);

  // A pick made while signed out runs once sign-in completes.
  useEffect(() => {
    if (user && pendingRef.current) {
      const code = pendingRef.current;
      pendingRef.current = null;
      handleLanguageSelect(code);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [user]);

  const restoreOriginals = () => {
    for (const [element, original] of originalsRef.current) {
      if (!element.isConnected) continue;
      element.innerHTML = original.html;
      if (original.langClass) element.classList.remove(original.langClass);
      element.removeAttribute("lang");
    }
    originalsRef.current.clear();
  };

  const handleLanguageSelect = async (code) => {
    setIsOpen(false);

    // Docusaurus passes onClick to every navbar item in the mobile
    // sidebar: it toggles the sidebar closed (desktop items get no
    // onClick). Close the hamburger for every pick, so the page the
    // language applies to — or the sign-in modal, which is portaled
    // to document.body — is actually visible instead of sitting
    // behind the open drawer.
    if (onNavbarItemClick) onNavbarItemClick();

    if (code === activeLanguage) return;

    // Translation needs a signed-in account. Remember the pick and
    // open sign-in; it runs when the user comes back signed in.
    if (!user && code !== "en") {
      pendingRef.current = code;
      setIsAuthModalOpen(true);
      return;
    }

    // A translation replaces the text being read aloud: stop the
    // Listen button before the words under it change.
    if (typeof window !== "undefined") {
      window.dispatchEvent(new CustomEvent(CANCEL_SPEECH_EVENT));
    }

    // A pick while a translation is running cancels it first; the
    // run id below keeps the old run's late callbacks from applying
    // over the new one.
    abortRef.current?.abort();
    const runId = ++runIdRef.current;

    // Back to the original text first, whatever the pick: switching
    // languages translates from the English original, never from an
    // already-translated page.
    restoreOriginals();

    if (code === "en") {
      setActiveLanguage("en");
      setCurrentLanguage("en");
      return;
    }

    const lang = getLanguage(code);
    setLoading(true);
    setProgress({ done: 0, total: 0 });

    const controller = new AbortController();
    abortRef.current = controller;

    const validElements = [];
    const textsToTranslate = [];
    // Declared here (not inside try) so the AbortError branch below
    // can apply the finished chunks too.
    const applyTranslations = (list, start = 0) => {
      if (runId !== runIdRef.current) return 0;
      let count = 0;
      (list || []).forEach((translation, idx) => {
        const element = validElements[start + idx];
        if (translation && element) {
          count += 1;
          applyRewrittenText(element, translation);
          // The script's own face and direction, if it needs one;
          // Latin languages keep Inter and the page's own alignment.
          if (lang.fontClass) element.classList.add(lang.fontClass);
          element.setAttribute("lang", lang.code);
        }
      });
      return count;
    };

    try {
      // Select all text content in the documentation.
      const collected = collectChapterElements(2);

      if (collected.length === 0) {
        alert("No content found to translate.");
        setLoading(false);
        return;
      }

      for (const { element, text } of collected) {
        validElements.push(element);
        textsToTranslate.push(text);
        if (!originalsRef.current.has(element)) {
          originalsRef.current.set(element, {
            html: element.innerHTML,
            langClass: lang.fontClass || null,
          });
        }
      }

      const total = textsToTranslate.length;

      // Translate a few elements per API call (the backend translates
      // one text per call and rate-limits AI requests). Chunks run
      // concurrently with a bound of 8 and cached chunks come back
      // from Neon — the cache keys on the language and the page, with
      // no user in it, so a page another reader already translated
      // comes back near-instant. Elements that could not be
      // translated keep their original English text.
      const translations = await translateTexts(
        textsToTranslate,
        (done, totalCount) => setProgress({ done, total: totalCount }),
        {
          signal: controller.signal,
          targetLanguage: lang.label,
          onChunkSettled: (start, chunkResults) => {
            applyTranslations(chunkResults, start);
          },
        }
      );

      const translatedCount = applyTranslations(translations);

      if (translatedCount === 0) {
        alert("Translation failed. The translation service is unavailable right now — please try again later.");
        setLoading(false);
        setProgress({ done: 0, total: 0 });
        return;
      }

      setActiveLanguage(code);
      setCurrentLanguage(code);
    } catch (error) {
      // A cancel is not a failure — apply the chunks that already
      // finished (runChunked carries them on the error as
      // partialResults) and say nothing. If anything was translated,
      // the page is (partly) in the new language.
      if (error?.name === "AbortError") {
        if (applyTranslations(error.partialResults) > 0) {
          setActiveLanguage(code);
          setCurrentLanguage(code);
        }
        return;
      }
      console.error(error);
      alert("Translation failed. The translation service is unavailable right now — please try again later.");
    } finally {
      if (runId === runIdRef.current) abortRef.current = null;
      setLoading(false);
      setProgress({ done: 0, total: 0 });
    }
  };

  const getLabel = () => {
    if (loading) return progress.total > 0 ? `${progress.done} of ${progress.total}` : "Translating…";
    return getLanguage(activeLanguage).nativeLabel;
  };

  return (
    <div className="language-dropdown" ref={dropdownRef}>
      <button
        className={`navbar-btn navbar-btn--secondary language-trigger ${loading ? "navbar-btn--loading" : ""}`}
        onClick={() => {
          // While a translation is running the trigger is the cancel
          // control — tapping it stops every in-flight request.
          if (loading) {
            abortRef.current?.abort();
            return;
          }
          setIsOpen(!isOpen);
        }}
        aria-expanded={isOpen}
        aria-haspopup="menu"
        aria-label={loading ? "Cancel translation" : `Language, currently ${getLanguage(activeLanguage).label}`}
        title={loading ? "Translating — tap to cancel" : "Choose a language"}
      >
        <span className="language-icon"><GlobeIcon /></span>
        <span className="navbar-btn__label">{getLabel()}</span>
        {loading ? (
          <span className="language-chevron language-cancel"><CancelIcon /></span>
        ) : (
          <span className="language-chevron"><ChevronDown /></span>
        )}
      </button>

      {isOpen && (
        <div className="language-menu" role="menu">
          {LANGUAGES.map((lang) => (
            <button
              key={lang.code}
              className="language-menu-item"
              onClick={() => handleLanguageSelect(lang.code)}
              role="menuitem"
              aria-current={activeLanguage === lang.code}
            >
              <span className="language-menu-text">
                <span className="language-menu-label">{lang.nativeLabel}</span>
                <span className="language-menu-sub">
                  {lang.code === "en" ? "Original text" : lang.label}
                </span>
              </span>
              {activeLanguage === lang.code && (
                <span className="language-menu-check"><CheckIcon /></span>
              )}
            </button>
          ))}
        </div>
      )}

      <AuthModal
        isOpen={isAuthModalOpen}
        onClose={() => setIsAuthModalOpen(false)}
      />
    </div>
  );
};

const UrduButton = (props) => {
  return (
    <BrowserOnly fallback={null}>
      {() => <LanguageMenuContent {...props} />}
    </BrowserOnly>
  );
};

export default UrduButton;
