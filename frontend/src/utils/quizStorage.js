/**
 * Quiz Storage Utility
 * Feature: 002-ui-improvements
 * Task: T072
 *
 * localStorage-based persistence for quiz progress and scores
 */

const STORAGE_PREFIX = 'quiz-';
const STORAGE_VERSION = 'v1';

/**
 * Save quiz progress
 */
export function saveQuizProgress(quizId, data) {
  if (typeof window === 'undefined') return false;

  try {
    const key = `${STORAGE_PREFIX}${STORAGE_VERSION}-${quizId}`;
    const payload = {
      ...data,
      savedAt: Date.now()
    };
    localStorage.setItem(key, JSON.stringify(payload));
    return true;
  } catch (e) {
    console.warn('Could not save quiz progress:', e);
    return false;
  }
}

/**
 * Load quiz progress
 */
export function loadQuizProgress(quizId) {
  if (typeof window === 'undefined') return null;

  try {
    const key = `${STORAGE_PREFIX}${STORAGE_VERSION}-${quizId}`;
    const data = localStorage.getItem(key);
    return data ? JSON.parse(data) : null;
  } catch (e) {
    console.warn('Could not load quiz progress:', e);
    return null;
  }
}

/**
 * Clear quiz progress
 */
export function clearQuizProgress(quizId) {
  if (typeof window === 'undefined') return false;

  try {
    const key = `${STORAGE_PREFIX}${STORAGE_VERSION}-${quizId}`;
    localStorage.removeItem(key);
    return true;
  } catch (e) {
    console.warn('Could not clear quiz progress:', e);
    return false;
  }
}

/**
 * Get all quiz scores
 */
export function getAllQuizScores() {
  if (typeof window === 'undefined') return {};

  try {
    const scores = {};
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key && key.startsWith(`${STORAGE_PREFIX}${STORAGE_VERSION}-`)) {
        const quizId = key.replace(`${STORAGE_PREFIX}${STORAGE_VERSION}-`, '');
        const data = JSON.parse(localStorage.getItem(key));
        if (data.completed) {
          scores[quizId] = {
            correct: data.score?.correct || 0,
            total: data.score?.total || 0,
            percentage: data.score?.percentage || 0,
            completedAt: data.completedAt || data.savedAt
          };
        }
      }
    }
    return scores;
  } catch (e) {
    console.warn('Could not get quiz scores:', e);
    return {};
  }
}

/**
 * Calculate quiz statistics across all quizzes
 */
export function getQuizStats() {
  const scores = getAllQuizScores();
  const quizIds = Object.keys(scores);

  if (quizIds.length === 0) {
    return {
      totalQuizzes: 0,
      totalQuestions: 0,
      totalCorrect: 0,
      averageScore: 0
    };
  }

  const totalQuestions = quizIds.reduce((sum, id) => sum + scores[id].total, 0);
  const totalCorrect = quizIds.reduce((sum, id) => sum + scores[id].correct, 0);
  const averageScore = totalCorrect / totalQuestions;

  return {
    totalQuizzes: quizIds.length,
    totalQuestions,
    totalCorrect,
    averageScore
  };
}

/**
 * Clear all quiz data
 */
export function clearAllQuizData() {
  if (typeof window === 'undefined') return false;

  try {
    const keysToRemove = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key && key.startsWith(`${STORAGE_PREFIX}${STORAGE_VERSION}-`)) {
        keysToRemove.push(key);
      }
    }
    keysToRemove.forEach(key => localStorage.removeItem(key));
    return true;
  } catch (e) {
    console.warn('Could not clear quiz data:', e);
    return false;
  }
}

/**
 * React hook for quiz progress
 */
export function useQuizProgress(quizId) {
  const [progress, setProgress] = React.useState(null);
  const [isLoading, setIsLoading] = React.useState(true);

  React.useEffect(() => {
    setProgress(loadQuizProgress(quizId));
    setIsLoading(false);
  }, [quizId]);

  const saveProgress = React.useCallback((data) => {
    const success = saveQuizProgress(quizId, data);
    if (success) {
      setProgress(data);
    }
    return success;
  }, [quizId]);

  const clearProgress = React.useCallback(() => {
    const success = clearQuizProgress(quizId);
    if (success) {
      setProgress(null);
    }
    return success;
  }, [quizId]);

  return {
    progress,
    isLoading,
    saveProgress,
    clearProgress
  };
}

// Import React for the hook
import React from 'react';
