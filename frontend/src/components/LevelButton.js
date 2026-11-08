/**
 * Level Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Content difficulty selector (Basic/Intermediate/Pro) with unified styling.
 */
import React, { useState } from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { useAuth } from "./Auth/AuthProvider";
import AuthModal from "./Auth/AuthModal";
import { personalizeTexts } from "../config/api";
import "./LevelButton.css";

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

const LevelButtonContent = () => {
  const { user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [currentLevel, setCurrentLevel] = useState("pro");
  const [loading, setLoading] = useState(false);
  const [progress, setProgress] = useState(0);

  // Load user's proficiency from profile on mount
  React.useEffect(() => {
    if (user && user.proficiency) {
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

  const handleLevelSelect = async (level) => {
    setIsOpen(false);

    // SECURITY CHECK: Require Login
    if (!user) {
      setIsAuthModalOpen(true);
      return;
    }

    // If PRO is selected, just reload to get original text back
    if (level === "pro") {
      if (currentLevel !== "pro") {
        window.location.reload();
      }
      return;
    }

    // If Basic or Intermediate, call AI
    setCurrentLevel(level);
    setLoading(true);
    setProgress(0);

    try {
      // Select content paragraphs (skipping code blocks)
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown li"
      );
      const elementsArray = Array.from(contentElements);

      // Filter valid elements and collect texts for batching
      const validElements = [];
      const textsToAdjust = [];

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
      // per call and rate-limits AI requests). Elements that could not be
      // rewritten keep their original text.
      const rewritten = await personalizeTexts(textsToAdjust, level, (pct) =>
        setProgress(pct)
      );

      let rewrittenCount = 0;
      rewritten.forEach((content, idx) => {
        const element = validElements[idx];
        if (content && element) {
          rewrittenCount += 1;
          element.innerText = content;

          // Visual styling based on level
          if (level === "beginner") {
            element.style.color = "#a8d5ff"; // Light Blue for Basic
            element.style.borderLeft = "3px solid #3b82f6";
            element.style.paddingLeft = "10px";
          } else {
            element.style.color = "#e9d5ff"; // Light Purple for Intermediate
            element.style.borderLeft = "3px solid #a855f7";
            element.style.paddingLeft = "10px";
          }
        }
      });

      if (rewrittenCount === 0) {
        alert("Could not rewrite the content. The AI service is unavailable right now — please try again later.");
        setCurrentLevel("pro");
        setLoading(false);
        return;
      }
    } catch (error) {
      alert("Could not rewrite the content. The AI service is unavailable right now — please try again later.");
      setCurrentLevel("pro");
    } finally {
      setLoading(false);
    }
  };

  const getLabel = () => {
    if (loading) return progress > 0 ? `${progress}%` : "Generating…";
    if (currentLevel === "beginner") return "Basic";
    if (currentLevel === "intermediate") return "Intermediate";
    return "Pro";
  };

  const getDotColor = () => {
    if (currentLevel === "beginner") return "#3b82f6"; // Blue
    if (currentLevel === "intermediate") return "#a855f7"; // Purple
    return "#2ECC71"; // Green
  };

  return (
    <div className="level-dropdown">
      <button
        className={`navbar-btn navbar-btn--accent level-trigger ${loading ? "navbar-btn--loading" : ""}`}
        onClick={() => !loading && setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="menu"
      >
        <span className="level-icon"><LevelIcon /></span>
        <span className="navbar-btn__label">{getLabel()}</span>
        {!loading && <span className="level-chevron"><ChevronDown /></span>}
        {!loading && <span className="level-dot" style={{ backgroundColor: getDotColor() }} />}
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

const LevelButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <LevelButtonContent />}</BrowserOnly>
  );
};

export default LevelButton;
