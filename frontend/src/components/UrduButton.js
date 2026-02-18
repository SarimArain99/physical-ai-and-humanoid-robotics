/**
 * Urdu Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Urdu/English language toggle with unified styling.
 */
import React, { useState } from "react";
import { useAuth } from "./Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";
import { contentUrls } from "../../config/api";
import "./UrduButton.css";

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

      // Process in batches of 10 for better performance
      const BATCH_SIZE = 10;
      let processed = 0;

      for (let batchStart = 0; batchStart < total; batchStart += BATCH_SIZE) {
        const batchEnd = Math.min(batchStart + BATCH_SIZE, total);
        const batchTexts = textsToTranslate.slice(batchStart, batchEnd);
        const batchElements = validElements.slice(batchStart, batchEnd);

        try {
          const response = await fetch(
            contentUrls.translateBatch(),
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
                contentUrls.translate(),
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

  const getLabel = () => {
    if (loading) return `${progress}%`;
    if (isUrdu) return "EN";
    return "اردو"; // Urdu in Urdu script
  };

  const getIcon = () => {
    if (isUrdu) return "🇬🇧"; // UK flag for English
    return "🇵🇰"; // Pakistan flag for Urdu
  };

  return (
    <button
      className={`navbar-btn navbar-btn--secondary urdu-button ${loading ? "navbar-btn--loading" : ""}`}
      onClick={handleTranslate}
      disabled={loading}
      title={isUrdu ? "Switch to English" : "Switch to Urdu translation"}
      aria-label={isUrdu ? "Switch to English" : "Switch to Urdu"}
    >
      <span className="urdu-flag">{getIcon()}</span>
      <span className="navbar-btn__label">{getLabel()}</span>
    </button>
  );
};

const UrduButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <UrduButtonContent />}</BrowserOnly>
  );
};

export default UrduButton;
