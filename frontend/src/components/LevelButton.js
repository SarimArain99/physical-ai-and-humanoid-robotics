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

      // Process in batches of 10 for better performance
      const BATCH_SIZE = 10;
      let processed = 0;

      for (let batchStart = 0; batchStart < total; batchStart += BATCH_SIZE) {
        const batchEnd = Math.min(batchStart + BATCH_SIZE, total);
        const batchTexts = textsToAdjust.slice(batchStart, batchEnd);
        const batchElements = validElements.slice(batchStart, batchEnd);

        try {
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/adjust-content/batch",
            {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                texts: batchTexts,
                target_level: level,
              }),
            }
          );

          const data = await response.json();

          if (data.contents && Array.isArray(data.contents)) {
            data.contents.forEach((content, idx) => {
              if (content && batchElements[idx]) {
                batchElements[idx].innerText = content;

                // Visual Styling based on level
                if (level === "beginner") {
                  batchElements[idx].style.color = "#a8d5ff"; // Light Blue for Basic
                  batchElements[idx].style.borderLeft = "3px solid #3b82f6";
                  batchElements[idx].style.paddingLeft = "10px";
                } else {
                  batchElements[idx].style.color = "#e9d5ff"; // Light Purple for Intermediate
                  batchElements[idx].style.borderLeft = "3px solid #a855f7";
                  batchElements[idx].style.paddingLeft = "10px";
                }
              }
            });
          }
        } catch (err) {
          console.error("Batch adjustment failed, falling back to individual:", err);
          // Fallback: adjust individually if batch fails
          for (let i = 0; i < batchTexts.length; i++) {
            try {
              const response = await fetch(
                "https://physical-ai-and-humanoid-robotics-production.up.railway.app/adjust-content",
                {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({
                    text: batchTexts[i],
                    target_level: level,
                  }),
                }
              );
              const data = await response.json();
              if (data.content && batchElements[i]) {
                batchElements[i].innerText = data.content;
                if (level === "beginner") {
                  batchElements[i].style.color = "#a8d5ff";
                  batchElements[i].style.borderLeft = "3px solid #3b82f6";
                  batchElements[i].style.paddingLeft = "10px";
                } else {
                  batchElements[i].style.color = "#e9d5ff";
                  batchElements[i].style.borderLeft = "3px solid #a855f7";
                  batchElements[i].style.paddingLeft = "10px";
                }
              }
            } catch (innerErr) {
              console.error("Individual adjustment failed", innerErr);
            }
          }
        }

        processed += batchTexts.length;
        setProgress(Math.round((processed / total) * 100));
      }
    } catch (error) {
      alert("Error adapting content.");
    } finally {
      setLoading(false);
    }
  };

  const getLabel = () => {
    if (loading) return `${progress}%`;
    if (currentLevel === "beginner") return "Basic";
    if (currentLevel === "intermediate") return "Int.";
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
