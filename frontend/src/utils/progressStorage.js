/**
 * Progress Storage Utility
 * Feature: 002-ui-improvements
 * Task: T041
 *
 * Manages course progress persistence using localStorage.
 * Tracks chapter completion, scroll position, and reading progress.
 */

import React from 'react';

const STORAGE_KEY = 'physical-ai-progress';
const SCROLL_POSITIONS_KEY = 'physical-ai-scroll-positions';

/**
 * Get all stored progress data
 */
export function getProgressData() {
  if (typeof window === 'undefined') return null;

  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    return stored ? JSON.parse(stored) : {};
  } catch (error) {
    console.error('Failed to read progress data:', error);
    return {};
  }
}

/**
 * Save progress data
 */
function saveProgressData(data) {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(data));
  } catch (error) {
    console.error('Failed to save progress data:', error);
  }
}

/**
 * Mark a chapter as completed
 *
 * @param {string} chapterPath - The path/ID of the chapter
 * @param {number} percentRead - Percentage of chapter read (0-100)
 */
export function markChapterProgress(chapterPath, percentRead = 100) {
  const data = getProgressData();

  if (!data.chapters) {
    data.chapters = {};
  }

  data.chapters[chapterPath] = {
    completed: percentRead >= 100,
    percentRead,
    lastReadAt: new Date().toISOString(),
    readCount: (data.chapters[chapterPath]?.readCount || 0) + (percentRead >= 100 ? 1 : 0)
  };

  saveProgressData(data);

  return data.chapters[chapterPath];
}

/**
 * Get chapter progress
 *
 * @param {string} chapterPath - The path/ID of the chapter
 * @returns {Object|null} Chapter progress data or null
 */
export function getChapterProgress(chapterPath) {
  const data = getProgressData();
  return data.chapters?.[chapterPath] || null;
}

/**
 * Check if a chapter is completed
 *
 * @param {string} chapterPath - The path/ID of the chapter
 * @returns {boolean} True if chapter is marked as completed
 */
export function isChapterCompleted(chapterPath) {
  const progress = getChapterProgress(chapterPath);
  return progress?.completed || false;
}

/**
 * Get overall course progress
 *
 * @returns {Object} Overall progress stats
 */
export function getCourseProgress() {
  const data = getProgressData();
  const chapters = data.chapters || {};

  const totalChapters = Object.keys(chapters).length;
  const completedChapters = Object.values(chapters).filter(c => c.completed).length;
  const totalPercentRead = Object.values(chapters).reduce((sum, c) => sum + (c.percentRead || 0), 0);
  const averagePercent = totalChapters > 0 ? Math.round(totalPercentRead / totalChapters) : 0;

  return {
    totalChapters,
    completedChapters,
    averagePercent,
    lastReadAt: data.lastReadAt || null
  };
}

/**
 * Save scroll position for a page
 *
 * @param {string} path - Page path
 * @param {number} scrollY - Scroll Y position
 * @param {number} percentRead - Percentage of page read
 */
export function saveScrollPosition(path, scrollY, percentRead = 0) {
  if (typeof window === 'undefined') return;

  try {
    const positions = JSON.parse(localStorage.getItem(SCROLL_POSITIONS_KEY) || '{}');
    positions[path] = {
      scrollY,
      percentRead,
      savedAt: new Date().toISOString()
    };
    localStorage.setItem(SCROLL_POSITIONS_KEY, JSON.stringify(positions));

    // Also update chapter progress if percentRead is significant
    if (percentRead >= 80) {
      markChapterProgress(path, percentRead);
    }
  } catch (error) {
    console.error('Failed to save scroll position:', error);
  }
}

/**
 * Get saved scroll position for a page
 *
 * @param {string} path - Page path
 * @returns {Object|null} Scroll position data or null
 */
export function getScrollPosition(path) {
  if (typeof window === 'undefined') return null;

  try {
    const positions = JSON.parse(localStorage.getItem(SCROLL_POSITIONS_KEY) || '{}');
    return positions[path] || null;
  } catch (error) {
    console.error('Failed to read scroll position:', error);
    return null;
  }
}

/**
 * Clear all progress data
 */
export function clearProgress() {
  if (typeof window === 'undefined') return;

  try {
    localStorage.removeItem(STORAGE_KEY);
    localStorage.removeItem(SCROLL_POSITIONS_KEY);
  } catch (error) {
    console.error('Failed to clear progress data:', error);
  }
}

/**
 * Export progress data as JSON
 *
 * @returns {string} JSON string of progress data
 */
export function exportProgress() {
  const data = getProgressData();
  const scrollData = getScrollPosition('*');

  return JSON.stringify({
    progress: data,
    scrollPositions: scrollData,
    exportedAt: new Date().toISOString()
  }, null, 2);
}

/**
 * Import progress data from JSON
 *
 * @param {string} jsonString - JSON string of progress data
 * @returns {boolean} True if import succeeded
 */
export function importProgress(jsonString) {
  try {
    const data = JSON.parse(jsonString);

    if (data.progress) {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(data.progress));
    }

    if (data.scrollPositions) {
      localStorage.setItem(SCROLL_POSITIONS_KEY, JSON.stringify(data.scrollPositions));
    }

    return true;
  } catch (error) {
    console.error('Failed to import progress data:', error);
    return false;
  }
}

/**
 * Hook to automatically track reading progress
 *
 * @param {string} chapterPath - Current chapter path
 * @returns {Object} Progress tracking functions and state
 */
export function useProgressTracker(chapterPath) {
  const [progress, setProgress] = React.useState(0);

  React.useEffect(() => {
    if (typeof window === 'undefined' || !chapterPath) return;

    // Restore saved scroll position on mount
    const savedPosition = getScrollPosition(chapterPath);
    if (savedPosition?.scrollY) {
      // Small delay to ensure content is loaded
      setTimeout(() => {
        window.scrollTo(0, savedPosition.scrollY);
      }, 100);
    }

    // Track scroll position
    let ticking = false;
    const handleScroll = () => {
      if (!ticking) {
        ticking = true;
        window.requestAnimationFrame(() => {
          const scrollTop = window.scrollY;
          const docHeight = document.documentElement.scrollHeight - window.innerHeight;
          const percentRead = Math.round((scrollTop / docHeight) * 100);

          setProgress(percentRead);
          saveScrollPosition(chapterPath, scrollTop, percentRead);

          ticking = false;
        });
      }
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    return () => window.removeEventListener('scroll', handleScroll);
  }, [chapterPath]);

  return { progress, markChapterProgress, getChapterProgress };
}
