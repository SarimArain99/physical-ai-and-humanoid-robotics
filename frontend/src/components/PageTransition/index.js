/**
 * Page Transition Component
 * Feature: 002-ui-improvements
 * Task: T045
 *
 * Provides smooth page transition animations when navigating between pages.
 * Supports fade, slide, and scale transitions with performance optimization.
 */

import React, { useEffect, useState, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import { areAnimationsEnabled } from '../../utils/performance';
import './styles.css';

/**
 * Available transition types
 */
const TRANSITION_TYPES = {
  FADE: 'fade',
  SLIDE_UP: 'slideUp',
  SLIDE_DOWN: 'slideDown',
  SLIDE_LEFT: 'slideLeft',
  SLIDE_RIGHT: 'slideRight',
  SCALE: 'scale',
  NONE: 'none'
};

/**
 * PageTransition Component
 *
 * Wraps children and applies transition animations on route changes.
 *
 * @param {Object} props
 * @param {React.ReactNode} props.children - Child elements to animate
 * @param {string} props.type - Transition type (default: 'fade')
 * @param {number} props.duration - Animation duration in ms (default: 300)
 * @param {boolean} props.disabled - Force disable transitions
 */
export default function PageTransition({
  children,
  type = 'fade',
  duration = 300,
  disabled = false
}) {
  const location = useLocation();
  const [isVisible, setIsVisible] = useState(false);
  const [currentKey, setCurrentKey] = useState(location.pathname);
  const containerRef = useRef(null);

  const animationsEnabled = !disabled && areAnimationsEnabled();
  const shouldAnimate = animationsEnabled && type !== TRANSITION_TYPES.NONE;

  useEffect(() => {
    if (!shouldAnimate) {
      setIsVisible(true);
      return;
    }

    // Trigger exit animation for old page
    if (currentKey !== location.pathname) {
      setIsVisible(false);

      // After exit animation, update key and trigger enter animation
      const exitTimer = setTimeout(() => {
        setCurrentKey(location.pathname);
        setIsVisible(true);

        // Scroll to top on page change
        if (containerRef.current) {
          window.scrollTo({ top: 0, behavior: 'instant' });
        }
      }, duration);

      return () => clearTimeout(exitTimer);
    }

    // Initial mount - trigger enter animation
    const enterTimer = setTimeout(() => {
      setIsVisible(true);
    }, 50);

    return () => clearTimeout(enterTimer);
  }, [location.pathname, currentKey, duration, shouldAnimate]);

  return (
    <div
      ref={containerRef}
      className={`page-transition page-transition-${type} ${isVisible ? 'page-transition-visible' : ''}`}
      style={{ '--transition-duration': `${duration}ms` }}
    >
      {children}
    </div>
  );
}

/**
 * TransitionType enum for type safety
 */
export const TransitionType = TRANSITION_TYPES;

/**
 * Hook to programmatically trigger page transitions
 */
export function usePageTransition() {
  const [isTransitioning, setIsTransitioning] = useState(false);

  const triggerTransition = (callback) => {
    setIsTransitioning(true);
    setTimeout(() => {
      if (callback) callback();
      setTimeout(() => setIsTransitioning(false), 300);
    }, 50);
  };

  return { isTransitioning, triggerTransition };
}

/**
 * RouteChangeSpinner - Shows a spinner during route changes
 */
export function RouteChangeSpinner({ show = false }) {
  if (!show) return null;

  return (
    <div className="route-change-spinner" aria-hidden="true">
      <div className="spinner-ring" />
    </div>
  );
}
