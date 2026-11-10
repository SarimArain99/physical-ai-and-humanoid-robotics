/**
 * Level Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Content difficulty selector (Basic/Intermediate/Pro) with unified styling.
 */
import React, { useState, useRef, useEffect } from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { useAuth } from "./Auth/AuthProvider";
import AuthModal from "./Auth/AuthModal";
import { personalizeTexts } from "../config/api";
import { applyRewrittenText } from "../utils/markdown";
import "./LevelButton.css";

// The level a visitor last picked, kept on their own device. The pill's
// default comes from the account's stored proficiency on the server
// (AuthProvider reads it from the user row, never from localStorage
// alone). A stored pick on this device wins over the server default;
// with neither, the original text ("pro") is the default. A pick is also
// saved back to the server, so the default follows the user to another
// device instead of reverting on reload.
const LEVEL_STORAGE_KEY = "physical-ai-level";
const VALID_LEVELS = ["beginner", "intermediate", "pro"];

function readStoredLevel() {
  try {
    const stored = localStorage.getItem(LEVEL_STORAGE_KEY);
    return VALID_LEVELS.includes(stored) ? stored : null;
  } catch {
    return null;
  }
}

function storeLevel(level) {
  try {
    localStorage.setItem(LEVEL_STORAGE_KEY, level);
  } catch {
    // Private mode etc. — the pill still updates for this page.
  }
}

// --- ICONS ---
const LevelIcon = () => (
  <svg
    width="18"
    height="18"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <line x1="12" y1="20" x2="12" y2="10"></line>
    <line x1="18" y1="20" x2="18" y2="4"></line>
    <line x1="6" y1="20" x2="6" y2="16"></line>
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
  >
    <path d="M6 9l6 6 6-6" />
  </svg>
);

// Shown in place of the chevron while a rewrite is running: the pill
// itself is the cancel control (tap it, or press Escape, to stop).
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

const LevelButtonContent = ({ onClick: onNavbarItemClick }) => {
  const { user, updateProfile } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [currentLevel, setCurrentLevel] = useState(() => readStoredLevel() || "pro");
  const [loading, setLoading] = useState(false);
  // Honest progress: texts actually rewritten out of the total on the
  // page — counts, never a percentage guessed from elapsed time.
  // Workstream C: chunks run concurrently (see api.js runChunked).
  const [progress, setProgress] = useState({ done: 0, total: 0 });
  const dropdownRef = useRef(null);
  // Workstream C: the in-flight rewrite, so tapping the pill or pressing
  // Escape while it runs cancels it instead of opening the menu.
  const abortRef = useRef(null);

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

  // Workstream C: Escape while a rewrite is running cancels it.
  useEffect(() => {
    if (!loading) return;
    const onKeyDown = (event) => {
      if (event.key === "Escape") abortRef.current?.abort();
    };
    document.addEventListener("keydown", onKeyDown);
    return () => document.removeEventListener("keydown", onKeyDown);
  }, [loading]);

  // Load the signed-in user's proficiency from the server on mount —
  // only when they have never picked a level on this device; a stored
  // pick wins. user.proficiency is the server value (AuthProvider), so
  // the pill defaults to what the account actually holds.
  React.useEffect(() => {
    if (user && user.proficiency && !readStoredLevel()) {
      const proficiencyMap = {
        "beginner": "beginner",
        "intermediate": "intermediate",
        "advanced": "pro",
        "pro": "pro"
      };
      const mappedLevel = proficiencyMap[user.proficiency] || "pro";
      setCurrentLevel(mappedLevel);
    }
  }, [user]);

  // Save an explicit pick to the account so it sticks on every device.
  // Best effort: the pill already updated locally, and a failed save
  // only means the server default stays as it was.
  const persistLevelPick = (level) => {
    if (!user) return;
    updateProfile({ proficiency: level }).catch(() => {});
  };

  const handleLevelSelect = async (level) => {
    setIsOpen(false);

    // Docusaurus passes onClick to every navbar item in the mobile
    // sidebar: it toggles the sidebar closed (desktop items get no
    // onClick). Close the hamburger for every pick, so the page the
    // level applies to — or the sign-in modal, which is portaled to
    // document.body — is actually visible instead of sitting behind
    // the open drawer. Before this the prop was silently dropped.
    if (onNavbarItemClick) onNavbarItemClick();

    // SECURITY CHECK: Require Login
    if (!user) {
      setIsAuthModalOpen(true);
      return;
    }

    // If PRO is selected, just reload to get original text back
    if (level === "pro") {
      storeLevel("pro");
      if (currentLevel !== "pro") {
        // Give the save a moment before the reload, but never block
        // the reload on a slow network.
        await Promise.race([
          updateProfile({ proficiency: "pro" }),
          new Promise((resolve) => setTimeout(resolve, 1000)),
        ]).catch(() => {});
        window.location.reload();
      }
      return;
    }

    // If Basic or Intermediate, call AI
    setCurrentLevel(level);
    storeLevel(level);
    persistLevelPick(level);
    setLoading(true);
    setProgress({ done: 0, total: 0 });

    // Workstream C: one controller for the whole rewrite, so the pill
    // (or Escape) can cancel every in-flight chunk request at once.
    const controller = new AbortController();
    abortRef.current = controller;

    // Declared here (not inside try) so the AbortError branch below can
    // apply the finished chunks too.
    const validElements = [];
    const textsToAdjust = [];
    // Fix round (2026-11-10): the reply is markdown, so render it —
    // the old code wrote element.innerText, which showed every "**"
    // literally. applyRewrittenText escapes first, then renders bold /
    // italic / code / links and drops a list marker the model echoes
    // for an <li>. `start` is the chunk's offset for progressive
    // application (see onChunkSettled below); the whole-list call uses 0.
    const applyRewritten = (list, start = 0) => {
      let count = 0;
      (list || []).forEach((content, idx) => {
        const element = validElements[start + idx];
        if (content && element) {
          count += 1;
          applyRewrittenText(element, content);

          // Mark adapted text with a hairline only: it keeps the
          // page's own colour and alignment in both themes.
          element.classList.add("level-rewritten");
        }
      });
      return count;
    };

    try {
      // Select content paragraphs (skipping code blocks)
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown li"
      );
      const elementsArray = Array.from(contentElements);

      // Filter valid elements and collect texts for batching
      for (const element of elementsArray) {
        const originalText = element.innerText;
        // Skip short text or code
        if (originalText.trim().length < 30) continue;
        if (element.closest("pre") || element.closest("code")) continue;

        validElements.push(element);
        textsToAdjust.push(originalText);
      }

      const total = textsToAdjust.length;
      if (total === 0) {
        setLoading(false);
        return;
      }

      // Rewrite a few elements per API call (the backend rewrites one text
      // per call and rate-limits AI requests). Chunks run concurrently
      // with a bound of 8 (Workstream C, fix round 2026-11-10) and cached
      // chunks come back from Neon, so a repeat visit is near-instant.
      // Elements that could not be rewritten keep their original text.
      // Fix round (2026-11-10): onChunkSettled applies each finished
      // chunk as soon as it lands, so the page transforms progressively
      // instead of holding the original text until the slowest chunk
      // finishes.
      const rewritten = await personalizeTexts(
        textsToAdjust,
        level,
        (done, totalCount) => setProgress({ done, total: totalCount }),
        {
          signal: controller.signal,
          onChunkSettled: (start, chunkResults) => {
            applyRewritten(chunkResults, start);
          },
        }
      );

      const rewrittenCount = applyRewritten(rewritten);

      if (rewrittenCount === 0) {
        alert("Could not rewrite the content. The AI service is unavailable right now — please try again later.");
        setCurrentLevel("pro");
        storeLevel("pro");
        setLoading(false);
        return;
      }
    } catch (error) {
      // Workstream C: a cancel is not a failure. Keep the picked level
      // and apply the chunks that already finished (runChunked carries
      // them on the error as partialResults) — finished work is not
      // thrown away just because the user stopped the rest. Say
      // nothing. Picking the level again resumes cheaply, because
      // finished chunks are already cached server-side.
      if (error?.name === "AbortError") {
        applyRewritten(error.partialResults);
        setLoading(false);
        return;
      }
      alert("Could not rewrite the content. The AI service is unavailable right now — please try again later.");
      setCurrentLevel("pro");
      storeLevel("pro");
    } finally {
      abortRef.current = null;
      setLoading(false);
    }
  };

  const getLabel = () => {
    // Workstream C: honest counts — texts rewritten out of the total on
    // the page. No percentage: a percentage would imply steady progress
    // against time, which chunked AI work does not have.
    if (loading) return progress.total > 0 ? `${progress.done} of ${progress.total}` : "Generating…";
    if (currentLevel === "beginner") return "Basic";
    if (currentLevel === "intermediate") return "Intermediate";
    return "Pro";
  };

  const getDotClass = () => {
    if (currentLevel === "beginner") return "level-dot--basic";
    if (currentLevel === "intermediate") return "level-dot--intermediate";
    return "level-dot--pro";
  };

  return (
    <div className="level-dropdown" ref={dropdownRef}>
      <button
        className={`navbar-btn navbar-btn--accent level-trigger ${loading ? "navbar-btn--loading" : ""}`}
        onClick={() => {
          // Workstream C: while a rewrite is running the pill is the
          // cancel control — tapping it stops every in-flight request.
          if (loading) {
            abortRef.current?.abort();
            return;
          }
          setIsOpen(!isOpen);
        }}
        aria-expanded={isOpen}
        aria-haspopup="menu"
        aria-label={loading ? "Cancel rewrite" : undefined}
        title={loading ? "Rewriting — tap to cancel" : undefined}
      >
        <span className="level-icon"><LevelIcon /></span>
        <span className="navbar-btn__label">{getLabel()}</span>
        <span className={`level-dot ${getDotClass()}`} />
        {loading ? (
          <span className="level-chevron level-cancel"><CancelIcon /></span>
        ) : (
          <span className="level-chevron"><ChevronDown /></span>
        )}
      </button>

      {isOpen && (
        <div className="level-menu" role="menu">
          <button
            className="level-menu-item"
            onClick={() => handleLevelSelect("beginner")}
            role="menuitem"
          >
            <span className="level-menu-dot level-menu-dot--basic"></span>
            <span className="level-menu-text">
              <span className="level-menu-label">Basic</span>
              <span className="level-menu-sub">Explain Like I'm 5</span>
            </span>
          </button>

          <button
            className="level-menu-item"
            onClick={() => handleLevelSelect("intermediate")}
            role="menuitem"
          >
            <span className="level-menu-dot level-menu-dot--intermediate"></span>
            <span className="level-menu-text">
              <span className="level-menu-label">Intermediate</span>
              <span className="level-menu-sub">For Students</span>
            </span>
          </button>

          <button
            className="level-menu-item"
            onClick={() => handleLevelSelect("pro")}
            role="menuitem"
          >
            <span className="level-menu-dot level-menu-dot--pro"></span>
            <span className="level-menu-text">
              <span className="level-menu-label">Pro</span>
              <span className="level-menu-sub">Original Research</span>
            </span>
          </button>
        </div>
      )}

      <AuthModal
        isOpen={isAuthModalOpen}
        onClose={() => setIsAuthModalOpen(false)}
      />
    </div>
  );
};

const LevelButton = (props) => {
  return (
    <BrowserOnly fallback={null}>
      {() => <LevelButtonContent {...props} />}
    </BrowserOnly>
  );
};

export default LevelButton;
