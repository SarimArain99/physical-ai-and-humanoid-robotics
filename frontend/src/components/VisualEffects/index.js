import React, { createContext, useContext, useEffect, useState, useCallback } from 'react';
import { ParticleBackground } from './ParticleBackground';
import { ScrollAnimations } from './ScrollAnimations';
import './styles.css';

/**
 * VisualEffects Provider Component
 * Feature: 002-ui-improvements
 * Task: T079, T086
 *
 * Provides advanced visual effects including:
 * - Particle backgrounds
 * - Scroll-triggered animations
 * - Performance-based quality adjustment
 * - Reduced motion support
 */

const VisualEffectsContext = createContext({
  enabled: true,
  performanceLevel: 'medium',
  reducedMotion: false
});

/**
 * Hook to access visual effects context
 */
export function useVisualEffects() {
  return useContext(VisualEffectsContext);
}

/**
 * Detect performance level
 */
function detectPerformanceLevel() {
  if (typeof window === 'undefined') return 'medium';

  // Check for reduced motion preference
  if (window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
    return 'low';
  }

  // Hardware detection
  const hardwareConcurrency = navigator.hardwareConcurrency || 2;
  const deviceMemory = navigator.deviceMemory || 4;

  if (hardwareConcurrency >= 8 && deviceMemory >= 8) return 'high';
  if (hardwareConcurrency >= 4 && deviceMemory >= 4) return 'medium';
  return 'low';
}

/**
 * Main VisualEffects Provider
 */
export default function VisualEffects({
  children,
  enableParticles = true,
  enableScrollAnimations = true,
  particleCount = 'auto',
  particleColor = 'rgba(46, 204, 113, 0.5)'
}) {
  const [performanceLevel, setPerformanceLevel] = useState('medium');
  const [reducedMotion, setReducedMotion] = useState(false);
  const [mounted, setMounted] = useState(false);

  // Detect capabilities on mount
  useEffect(() => {
    setPerformanceLevel(detectPerformanceLevel());

    // Check for reduced motion
    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
    setReducedMotion(mediaQuery.matches);

    // Listen for changes
    const handleChange = (e) => setReducedMotion(e.matches);
    mediaQuery.addEventListener('change', handleChange);

    setMounted(true);

    return () => mediaQuery.removeEventListener('change', handleChange);
  }, []);

  // Calculate particle count based on performance
  const getParticleCount = useCallback(() => {
    if (particleCount !== 'auto') return parseInt(particleCount);

    switch (performanceLevel) {
      case 'high': return 100;
      case 'medium': return 50;
      case 'low': return 20;
      default: return 50;
    }
  }, [particleCount, performanceLevel]);

  // Disable effects if reduced motion or very low performance
  const enabled = !reducedMotion && performanceLevel !== 'very-low';

  const contextValue = {
    enabled,
    performanceLevel,
    reducedMotion
  };

  return (
    <VisualEffectsContext.Provider value={contextValue}>
      {enabled && mounted && (
        <>
          {enableParticles && (
            <ParticleBackground
              count={getParticleCount()}
              color={particleColor}
            />
          )}
          {enableScrollAnimations && (
            <ScrollAnimations />
          )}
        </>
      )}
      {children}
    </VisualEffectsContext.Provider>
  );
}
