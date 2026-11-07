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
import { translateTexts } from "../config/api";
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

      // Translate a few elements per API call (the backend translates one
      // text per call and rate-limits AI requests). Elements that could
      // not be translated keep their original English text.
      const translations = await translateTexts(textsToTranslate, (pct) =>
        setProgress(pct)
      );

      let translatedCount = 0;
      translations.forEach((translation, idx) => {
        const element = validElements[idx];
        if (translation && element) {
          translatedCount += 1;
          element.innerText = translation;
          // Apply Urdu styling
          element.style.direction = "rtl";
          element.style.fontFamily =
            "'Jameel Noori Nastaleeq', 'Noto Nastaliq Urdu', serif";
          element.style.textAlign = "right";
          element.style.color = "#2ECC71";
          element.style.fontSize = "1.2em";
          element.style.lineHeight = "1.8";
        }
      });

      if (translatedCount === 0) {
        alert("Translation failed. The translation service is unavailable right now — please try again later.");
        setLoading(false);
        setProgress(0);
        return;
      }

      setIsUrdu(true);
    } catch (error) {
      console.error(error);
      alert("Translation failed. The translation service is unavailable right now — please try again later.");
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
