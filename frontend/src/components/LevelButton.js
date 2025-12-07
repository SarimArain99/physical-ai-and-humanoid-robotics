import React, { useState } from "react";
import { useAuth } from "./Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly"; // 🟢 Import

// 1. Logic Component
const LevelButtonContent = () => {
  const { user } = useAuth();
  const [loading, setLoading] = useState(false);
  const [isSimplified, setIsSimplified] = useState(false);

  // If not logged in or is Pro, render nothing
  if (!user || !user.proficiency || user.proficiency === "pro") {
    return null;
  }

  const handleAdjust = async () => {
    if (isSimplified) {
      window.location.reload();
      return;
    }
    setLoading(true);

    try {
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown li"
      );
      const elementsArray = Array.from(contentElements);

      for (let i = 0; i < elementsArray.length; i++) {
        const element = elementsArray[i];
        const originalText = element.innerText;
        if (originalText.trim().length < 20) continue;

        try {
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/adjust-content",
            {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({
                text: originalText,
                target_level: user.proficiency,
              }),
            }
          );
          const data = await response.json();
          if (data.content) {
            element.innerText = data.content;
            element.style.color = "#a8d5ff";
            element.style.borderLeft = "3px solid #8b5cf6";
            element.style.paddingLeft = "10px";
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
    <button
      className={`level-button ${loading ? "loading" : ""}`}
      onClick={handleAdjust}
      disabled={loading}
    >
      {loading
        ? "Simplifying..."
        : isSimplified
        ? "↺ Original"
        : labels[user.proficiency] || "⚡ Adapt"}
    </button>
  );
};

// 2. Safe Wrapper
const LevelButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <LevelButtonContent />}</BrowserOnly>
  );
};

export default LevelButton;
