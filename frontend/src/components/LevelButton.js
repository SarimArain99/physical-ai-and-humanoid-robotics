import React, { useState } from "react";
import { useAuth } from "./Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";

const LevelButtonContent = () => {
  const { user } = useAuth();
  const [loading, setLoading] = useState(false);
  const [isSimplified, setIsSimplified] = useState(false);

  // Hide button if not logged in OR if user is already a "Pro"
  if (!user || !user.proficiency || user.proficiency === "pro") {
    return null;
  }

  const handleAdjust = async () => {
    // If already simplified, reload to reset to original text
    if (isSimplified) {
      window.location.reload();
      return;
    }

    setLoading(true);

    try {
      // Select paragraphs and list items
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown li"
      );
      const elementsArray = Array.from(contentElements);

      for (let i = 0; i < elementsArray.length; i++) {
        const element = elementsArray[i];
        const originalText = element.innerText;

        // Skip short text to save API calls
        if (originalText.trim().length < 20) continue;
        if (element.closest("pre") || element.closest("code")) continue;

        try {
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/adjust-content",
            {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
                // 'Authorization': `Bearer ${localStorage.getItem("auth_token")}`
              },
              body: JSON.stringify({
                text: originalText,
                target_level: user.proficiency,
              }),
            }
          );

          const data = await response.json();
          if (data.content) {
            element.innerText = data.content;
            // Visual cues that text has been adapted
            element.style.color = "#a8d5ff";
            element.style.borderLeft = "3px solid #8b5cf6";
            element.style.paddingLeft = "10px";
            element.style.transition = "all 0.5s ease";
          }
        } catch (err) {
          console.error(err);
        }
      }
      setIsSimplified(true);
    } finally {
      setLoading(false);
    }
  };

  const labels = {
    beginner: "👶 Explain Like I'm 5",
    intermediate: "🎓 Explain for Student",
  };

  return (
    <>
      <button
        className={`level-button ${loading ? "loading" : ""}`}
        onClick={handleAdjust}
        disabled={loading}
      >
        {loading
          ? "Thinking..."
          : isSimplified
          ? "↺ Original"
          : labels[user.proficiency] || "⚡ Adapt"}
      </button>

      {/* 🟢 EMBEDDED STYLES */}
      <style>{`
        .level-button {
          margin-left: 10px;
          padding: 6px 14px;
          background-color: #8b5cf6; /* Violet/Purple */
          color: white;
          border: none;
          border-radius: 20px;
          cursor: pointer;
          font-weight: bold;
          font-size: 13px;
          transition: transform 0.2s, box-shadow 0.2s;
          box-shadow: 0 2px 10px rgba(139, 92, 246, 0.3);
          display: inline-flex;
          align-items: center;
          white-space: nowrap;
        }
        .level-button:hover {
          background-color: #7c3aed;
          transform: scale(1.05);
          box-shadow: 0 4px 15px rgba(139, 92, 246, 0.5);
        }
        .level-button.loading {
          opacity: 0.8;
          cursor: wait;
          background-color: #6d28d9;
        }
      `}</style>
    </>
  );
};

// 🛡️ Safe Wrapper for Vercel Build
const LevelButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <LevelButtonContent />}</BrowserOnly>
  );
};

export default LevelButton;
