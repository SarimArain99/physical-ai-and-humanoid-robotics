import React, { useState } from "react";
import { useAuth } from "./Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";

const UrduButtonContent = () => {
  const { user } = useAuth();
  const [isUrdu, setIsUrdu] = useState(false);
  const [loading, setLoading] = useState(false);
  const [progress, setProgress] = useState(0);

  // Hide button if user is not logged in
  if (!user) return null;

  const handleTranslate = async () => {
    // If already translated, reload to reset to English
    if (isUrdu) {
      window.location.reload();
      return;
    }

    setLoading(true);
    setProgress(0);

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
      const total = elementsArray.length;

      for (let i = 0; i < total; i++) {
        const element = elementsArray[i];
        const originalText = element.innerText;

        // Skip very short text or code blocks
        if (originalText.trim().length < 2) continue;
        if (element.closest("pre") || element.closest("code")) continue;

        try {
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/translate",
            {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
                // Optional: Send token if you want to track usage later
                // 'Authorization': `Bearer ${localStorage.getItem("auth_token")}`
              },
              body: JSON.stringify({ text: originalText }),
            }
          );

          const data = await response.json();

          if (data.translation) {
            element.innerText = data.translation;
            // Apply Urdu Styling
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

        // Update progress bar
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
    <>
      <button
        className={`urdu-button ${loading ? "loading" : ""}`}
        onClick={handleTranslate}
        disabled={loading}
      >
        {loading
          ? `Translating ${progress}%`
          : isUrdu
          ? "US-English"
          : "PK-Urdu"}
      </button>

      {/* 🟢 EMBEDDED STYLES: This forces the style to load immediately */}
      <style>{`
        .urdu-button {
          margin-left: 10px;
          padding: 6px 14px;
          border: 1px solid #2ECC71;
          background-color: transparent;
          color: #2ECC71;
          border-radius: 20px;
          cursor: pointer;
          font-weight: bold;
          font-size: 13px;
          transition: all 0.2s;
          display: inline-flex;
          align-items: center;
          white-space: nowrap;
        }
        .urdu-button:hover {
          background-color: #2ECC71;
          color: #1E2A38;
          transform: translateY(-1px);
        }
        .urdu-button.loading {
          opacity: 0.7;
          cursor: wait;
          border-color: #27ae60;
        }
      `}</style>
    </>
  );
};

// 🛡️ Safe Wrapper for Vercel Build
const UrduButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <UrduButtonContent />}</BrowserOnly>
  );
};

export default UrduButton;
