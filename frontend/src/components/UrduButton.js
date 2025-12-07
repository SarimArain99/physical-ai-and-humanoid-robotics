import React, { useState } from "react";
// 1. Import the Auth Hook
import { useAuth } from "./Auth/AuthProvider";

const UrduButton = () => {
  // 2. Get the current user status
  const { user } = useAuth();

  const [isUrdu, setIsUrdu] = useState(false);
  const [loading, setLoading] = useState(false);
  const [progress, setProgress] = useState(0);

  // 3. SECURITY CHECK: If no user, hide the button completely
  if (!user) {
    return null;
  }

  const handleTranslate = async () => {
    if (isUrdu) {
      window.location.reload();
      return;
    }

    setLoading(true);
    setProgress(0);

    try {
      const contentElements = document.querySelectorAll(
        ".markdown p, .markdown h1, .markdown h2, .markdown h3, .markdown li"
      );

      if (contentElements.length === 0) {
        alert("No content found to translate.");
        setLoading(false);
        return;
      }

      const elementsArray = Array.from(contentElements);
      const total = elementsArray.length;

      for (let i = 0; i < total; i++) {
        const element = elementsArray[i];
        const originalText = element.innerText;

        if (originalText.trim().length < 2) continue;

        try {
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/translate",
            {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ text: originalText }),
            }
          );

          const data = await response.json();

          if (data.translation) {
            element.innerText = data.translation;
            element.style.direction = "rtl";
            element.style.fontFamily =
              "'Jameel Noori Nastaleeq', 'Noto Nastaliq Urdu', serif";
            element.style.textAlign = "right";
            element.style.color = "#2ECC71";
            element.style.fontSize = "1.2em";
            element.style.lineHeight = "1.8";
          }
        } catch (err) {
          console.error("Chunk failed", err);
        }

        setProgress(Math.round(((i + 1) / total) * 100));
      }

      setIsUrdu(true);
    } catch (error) {
      console.error(error);
      alert("Translation failed. Check backend.");
    } finally {
      setLoading(false);
      setProgress(0);
    }
  };

  return (
    <button
      className={`urdu-button ${loading ? "loading" : ""}`}
      onClick={handleTranslate}
      disabled={loading}
    >
      {loading
        ? `Translating ${progress}%...`
        : isUrdu
        ? "🇺🇸 English"
        : "🇵🇰 Urdu"}
    </button>
  );
};

export default UrduButton;
