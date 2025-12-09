import React, { useState } from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { useAuth } from "./Auth/AuthProvider"; // 🟢 Import Auth
import AuthModal from "./Auth/AuthModal"; // 🟢 Import Modal

// --- ICONS ---
const LevelIcon = () => (
  <svg
    width="20"
    height="20"
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
    width="14"
    height="14"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="3"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <path d="M6 9l6 6 6-6" />
  </svg>
);

const LevelButtonContent = () => {
  const { user } = useAuth(); // 🟢 Get User Status
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false); // 🟢 Modal State
  const [currentLevel, setCurrentLevel] = useState("pro");
  const [loading, setLoading] = useState(false);
  const [progress, setProgress] = useState(0);

  const handleLevelSelect = async (level) => {
    setIsOpen(false);

    // 🟢 1. SECURITY CHECK: Require Login
    if (!user) {
      setIsAuthModalOpen(true);
      return;
    }

    // 2. If PRO is selected, just reload to get original text back
    if (level === "pro") {
      if (currentLevel !== "pro") {
        window.location.reload();
      }
      return;
    }

    // 3. If Basic or Intermediate, call AI
    setCurrentLevel(level);
    setLoading(true);
    setProgress(0);

    try {
      // Select content paragraphs (skipping code blocks)
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown li"
      );
      const elementsArray = Array.from(contentElements);
      const total = elementsArray.length;

      for (let i = 0; i < total; i++) {
        const element = elementsArray[i];
        const originalText = element.innerText;

        // Skip short text or code
        if (originalText.trim().length < 30) continue;
        if (element.closest("pre") || element.closest("code")) continue;

        try {
          // Call your Backend
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/adjust-content",
            {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                text: originalText,
                target_level: level, // 'beginner' (Basic) or 'intermediate'
              }),
            }
          );

          const data = await response.json();
          if (data.content) {
            element.innerText = data.content;

            // Visual Styling based on level
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
        } catch (err) {
          console.error(err);
        }

        // Update progress bar
        setProgress(Math.round(((i + 1) / total) * 100));
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
    if (currentLevel === "intermediate") return "Inter.";
    return "Pro";
  };

  return (
    <>
      <div className="level-dropdown-container">
        <button
          className={`level-trigger-btn ${loading ? "loading" : ""}`}
          onClick={() => !loading && setIsOpen(!isOpen)}
        >
          <LevelIcon />
          <span>{getLabel()}</span>
          {!loading && <ChevronDown />}
        </button>

        {isOpen && (
          <div className="level-menu">
            <button
              className="level-item"
              onClick={() => handleLevelSelect("beginner")}
            >
              <span className="dot basic"></span> Basic
              <span className="sub"> (Explain Like I'm 5)</span>
            </button>

            <button
              className="level-item"
              onClick={() => handleLevelSelect("intermediate")}
            >
              <span className="dot intermediate"></span> Intermediate
              <span className="sub"> (For Students)</span>
            </button>

            <button
              className="level-item pro"
              onClick={() => handleLevelSelect("pro")}
            >
              <span className="dot pro"></span> Pro
              <span className="sub"> (Original Research)</span>
            </button>
          </div>
        )}
      </div>

      {/* 🟢 AUTH POPUP (Hidden by default) */}
      <AuthModal
        isOpen={isAuthModalOpen}
        onClose={() => setIsAuthModalOpen(false)}
      />

      {/* EMBEDDED STYLES */}
      <style>{`
        .level-dropdown-container {
          position: relative;
          display: inline-block;
          margin-left: 10px;
        }

        /* Trigger Button */
        .level-trigger-btn {
          display: flex;
          align-items: center;
          gap: 6px;
          padding: 6px 12px;
          background-color: #1e2a38;
          border: 1px solid #8b5cf6;
          color: #8b5cf6;
          border-radius: 20px;
          cursor: pointer;
          font-weight: bold;
          font-size: 13px;
          transition: all 0.2s;
          height: 36px;
        }
        .level-trigger-btn:hover {
          background-color: #8b5cf6;
          color: white;
        }
        .level-trigger-btn.loading {
          opacity: 0.8;
          cursor: wait;
          background-color: #1e2a38;
          color: #8b5cf6;
        }

        /* Dropdown Menu */
        .level-menu {
          position: absolute;
          top: 110%;
          right: 0;
          width: 200px;
          background-color: #151e29;
          border: 1px solid #2c3e50;
          border-radius: 12px;
          box-shadow: 0 10px 30px rgba(0,0,0,0.5);
          overflow: hidden;
          z-index: 100;
          animation: slideDown 0.2s ease;
        }

        .level-item {
          display: flex;
          align-items: center;
          width: 100%;
          padding: 12px 16px;
          background: none;
          border: none;
          color: white;
          text-align: left;
          cursor: pointer;
          font-size: 14px;
          border-bottom: 1px solid #1e2a38;
        }
        .level-item:last-child { border-bottom: none; }
        .level-item:hover { background-color: #1e2a38; }

        .sub {
          font-size: 11px;
          color: #94a3b8;
          margin-left: 4px;
        }

        /* Colored Dots */
        .dot { width: 8px; height: 8px; border-radius: 50%; margin-right: 8px; }
        .dot.basic { background-color: #3b82f6; }        /* Blue */
        .dot.intermediate { background-color: #a855f7; } /* Purple */
        .dot.pro { background-color: #2ecc71; }          /* Green */

        @keyframes slideDown {
            from { opacity: 0; transform: translateY(-10px); }
            to { opacity: 1; transform: translateY(0); }
        }
      `}</style>
    </>
  );
};

const LevelButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <LevelButtonContent />}</BrowserOnly>
  );
};

export default LevelButton;
