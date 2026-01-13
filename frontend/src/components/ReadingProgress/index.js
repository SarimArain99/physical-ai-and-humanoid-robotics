/**
 * Reading Progress Component
 * Feature: 002-ui-improvements
 * Task: Part of US4 (T037-T038)
 *
 * Displays a horizontal progress bar at the top of the content area
 * showing the user's reading position within the current chapter.
 */

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { areAnimationsEnabled } from '../../utils/performance';
import { markChapterProgress } from '../../utils/progressStorage';
import './styles.css';

/**
 * ReadingProgress Component
 *
 * Shows a progress bar at the top of the viewport that fills
 * as the user scrolls through the content.
 *
 * @param {Object} props
 * @param {number} props.height - Height of the progress bar in pixels (default: 4)
 * @param {string} props.color - Color override (defaults to module primary)
 * @param {boolean} props.showLabel - Whether to show percentage text (default: false)
 * @param {Function} props.onComplete - Callback when progress reaches 100%
 * @param {boolean} props.throttle - Throttle scroll events for performance (default: true)
 */
export default function ReadingProgress({
  height = 4,
  color,
  showLabel = false,
  onComplete,
  throttle = true
}) {
  const [progress, setProgress] = useState(0);
  const [isComplete, setIsComplete] = useState(false);
  const location = useLocation();
  const animationsEnabled = areAnimationsEnabled();

  useEffect(() => {
    // Reset on route change
    setProgress(0);
    setIsComplete(false);
  }, [location.pathname]);

  useEffect(() => {
    let ticking = false;
    let hasSavedCompletion = false; // T044: Track if we've saved completion to avoid duplicate saves

    const handleScroll = () => {
      if (!ticking) {
        window.requestAnimationFrame(() => {
          const windowHeight = window.innerHeight;
          const documentHeight = document.documentElement.scrollHeight;
          const scrolled = window.scrollY;
          const maxScroll = documentHeight - windowHeight;
          const percent = maxScroll > 0 ? Math.min((scrolled / maxScroll) * 100, 100) : 0;

          setProgress(percent);

          // Check for completion
          if (percent >= 100 && !isComplete) {
            setIsComplete(true);

            // T044: Save completion to localStorage on 100% scroll
            if (!hasSavedCompletion) {
              markChapterProgress(location.pathname, 100);
              hasSavedCompletion = true;

              // Dispatch custom event for other components to listen
              window.dispatchEvent(new CustomEvent('chapter-completed', {
                detail: { path: location.pathname, timestamp: Date.now() }
              }));
            }

            if (onComplete) {
              onComplete();
            }
          } else if (percent < 100 && isComplete) {
            setIsComplete(false);
            hasSavedCompletion = false;
          }

          ticking = false;
        });

        ticking = true;
      }
    };

    // Add scroll listener
    window.addEventListener('scroll', handleScroll, { passive: true });

    // Initial check
    handleScroll();

    return () => {
      window.removeEventListener('scroll', handleScroll);
    };
  }, [isComplete, onComplete, location.pathname]);

  const barStyle = {
    height: `${height}px`,
    backgroundColor: color || 'var(--progress-color, var(--module-primary, #2ECC71))'
  };

  const fillStyle = {
    width: animationsEnabled ? `${progress}%` : '100%',
    transition: animationsEnabled ? 'width 0.1s linear' : 'none',
    backgroundColor: color || 'var(--progress-color, var(--module-primary, #2ECC71))'
  };

  return (
    <>
      <div className="reading-progress-bar" style={barStyle}>
        <div
          className="reading-progress-fill"
          style={fillStyle}
          role="progressbar"
          aria-valuenow={Math.round(progress)}
          aria-valuemin="0"
          aria-valuemax="100"
          aria-label="Reading progress"
        />
      </div>

      {showLabel && (
        <div className="reading-progress-label">
          {Math.round(progress)}%
        </div>
      )}
    </>
  );
}

/**
 * CourseProgress Component
 *
 * Shows overall course completion based on localStorage data.
 */
export function CourseProgress({ className = '' }) {
  const [courseProgress, setCourseProgress] = useState(0);

  useEffect(() => {
    // Load progress from localStorage
    const loadProgress = () => {
      try {
        const stored = localStorage.getItem('course-progress');
        if (stored) {
          const progress = JSON.parse(stored);
          const chapters = Object.values(progress);
          const completed = chapters.filter(c => c.completed).length;
          const total = chapters.length;
          setCourseProgress(total > 0 ? Math.round((completed / total) * 100) : 0);
        }
      } catch (e) {
        console.error('Failed to load course progress:', e);
      }
    };

    loadProgress();

    // Listen for storage changes
    window.addEventListener('storage', loadProgress);
    return () => window.removeEventListener('storage', loadProgress);
  }, []);

  if (courseProgress === 0) return null;

  return (
    <div className={`course-progress ${className}`}>
      <div className="course-progress-label">Course Progress</div>
      <div className="course-progress-bar">
        <div
          className="course-progress-fill"
          style={{ width: `${courseProgress}%` }}
        />
      </div>
      <div className="course-progress-percent">{courseProgress}%</div>
    </div>
  );
}

/**
 * useReadingProgress Hook
 *
 * Custom hook to access reading progress in components.
 */
export function useReadingProgress() {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const windowHeight = window.innerHeight;
      const documentHeight = document.documentElement.scrollHeight;
      const scrolled = window.scrollY;
      const maxScroll = documentHeight - windowHeight;
      const percent = maxScroll > 0 ? Math.min((scrolled / maxScroll) * 100, 100) : 0;
      setProgress(percent);
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    handleScroll();

    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return progress;
}
