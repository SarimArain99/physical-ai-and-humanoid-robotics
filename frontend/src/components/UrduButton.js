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

      // Filter valid elements and collect texts for batching
      const validElements = [];
      const textsToTranslate = [];

      for (const element of elementsArray) {
        const originalText = element.innerText;
        // Skip very short text or code blocks
        if (originalText.trim().length < 2) continue;
        if (element.closest("pre") || element.closest("code")) continue;

        validElements.push(element);
        textsToTranslate.push(originalText);
      }

      const total = textsToTranslate.length;
      if (total === 0) {
        alert("No translatable content found.");
        setLoading(false);
        return;
      }

      // Process in batches of 10 for better performance (T137)
      const BATCH_SIZE = 10;
      let processed = 0;

      for (let batchStart = 0; batchStart < total; batchStart += BATCH_SIZE) {
        const batchEnd = Math.min(batchStart + BATCH_SIZE, total);
        const batchTexts = textsToTranslate.slice(batchStart, batchEnd);
        const batchElements = validElements.slice(batchStart, batchEnd);

        try {
          const response = await fetch(
            "https://physical-ai-and-humanoid-robotics-production.up.railway.app/translate/batch",
            {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({ texts: batchTexts }),
            }
          );

          const data = await response.json();

          if (data.translations && Array.isArray(data.translations)) {
            // Apply translations to elements
            data.translations.forEach((translation, idx) => {
              if (translation && batchElements[idx]) {
                batchElements[idx].innerText = translation;
                // Apply Urdu Styling
                batchElements[idx].style.direction = "rtl";
                batchElements[idx].style.fontFamily =
                  "'Jameel Noori Nastaleeq', 'Noto Nastaliq Urdu', serif";
                batchElements[idx].style.textAlign = "right";
                batchElements[idx].style.color = "#2ECC71";
                batchElements[idx].style.fontSize = "1.2em";
                batchElements[idx].style.lineHeight = "1.8";
              }
            });
          }
        } catch (err) {
          console.error("Batch translation failed, falling back to individual:", err);
          // Fallback: translate individually if batch fails
          for (let i = 0; i < batchTexts.length; i++) {
            try {
              const response = await fetch(
                "https://physical-ai-and-humanoid-robotics-production.up.railway.app/translate",
                {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({ text: batchTexts[i] }),
                }
              );
              const data = await response.json();
              if (data.translation && batchElements[i]) {
                batchElements[i].innerText = data.translation;
                batchElements[i].style.direction = "rtl";
                batchElements[i].style.fontFamily = "'Jameel Noori Nastaleeq', 'Noto Nastaliq Urdu', serif";
                batchElements[i].style.textAlign = "right";
                batchElements[i].style.color = "#2ECC71";
                batchElements[i].style.fontSize = "1.2em";
                batchElements[i].style.lineHeight = "1.8";
              }
            } catch (innerErr) {
              console.error("Individual translation failed", innerErr);
            }
          }
        }

        processed += batchTexts.length;
        setProgress(Math.round((processed / total) * 100));
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
