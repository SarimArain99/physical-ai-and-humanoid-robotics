import React, { useState } from "react";
import { useAuth } from "./Auth/AuthProvider";

const LevelButton = () => {
  const { user } = useAuth();
  const [loading, setLoading] = useState(false);
  const [isSimplified, setIsSimplified] = useState(false);

  // Hide if not logged in OR if user is already Pro
  if (!user || !user.proficiency || user.proficiency === "pro") {
    return null;
  }

  const handleAdjust = async () => {
    if (isSimplified) {
      window.location.reload(); // Reset to original text
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
        // Skip short text
        if (originalText.trim().length < 20) continue;

        try {
          const response = await fetch("http://localhost:8000/adjust-content", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              text: originalText,
              target_level: user.proficiency,
            }),
          });

          const data = await response.json();
          if (data.content) {
            element.innerText = data.content;
            element.style.color = "#a8d5ff"; // Visual cue
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

export default LevelButton;
