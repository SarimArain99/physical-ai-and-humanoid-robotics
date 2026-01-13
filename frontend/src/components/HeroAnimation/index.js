/**
 * Hero Animation Component
 * Feature: 002-ui-improvements
 * Task: T015
 *
 * Provides staggered animation reveal for hero section elements
 * on page load. Supports animation disabling via performance
 * detection and reduced motion preferences.
 */

import React, { useEffect, useRef, useState } from 'react';
import { areAnimationsEnabled } from '../../utils/performance';
import './styles.css';

/**
 * HeroAnimation Component
 *
 * Wraps children and applies staggered fade/slide animations
 * on mount. Each child can have a custom delay.
 *
 * @param {Object} props
 * @param {React.ReactNode} props.children - Child elements to animate
 * @param {Array} props.elements - Array of element configs { selector, delay, animation }
 * @param {boolean} props.disabled - Force disable animations
 * @param {number} props.staggerDelay - Delay between each element (default: 100ms)
 * @param {string} props.animation - Default animation type (default: 'slideUp')
 */
export default function HeroAnimation({
  children,
  elements = [],
  disabled = false,
  staggerDelay = 100,
  animation = 'slideUp'
}) {
  const containerRef = useRef(null);
  const [shouldAnimate, setShouldAnimate] = useState(false);

  useEffect(() => {
    // Check if animations should be enabled
    const animationsEnabled = areAnimationsEnabled();
    setShouldAnimate(!disabled && animationsEnabled);

    if (disabled || !animationsEnabled) {
      return;
    }

    // Apply animations to children
    const childElements = containerRef.current?.children;
    if (!childElements) return;

    Array.from(childElements).forEach((child, index) => {
      const delay = elements[index]?.delay || index * staggerDelay;
      const animType = elements[index]?.animation || animation;

      // Set initial state
      child.style.opacity = '0';
      child.style.animationDelay = `${delay}ms`;
      child.classList.add(`hero-animate-${animType}`);

      // Trigger animation after a small delay to ensure DOM is ready
      setTimeout(() => {
        child.classList.add('hero-animate-visible');
      }, 50);
    });
  }, [disabled, elements, staggerDelay, animation]);

  return (
    <div
      ref={containerRef}
      className={`hero-animation-container ${shouldAnimate ? 'hero-enabled' : 'hero-disabled'}`}
      role="region"
      aria-label="Hero content"
      aria-live="polite"
    >
      {children}
    </div>
  );
}

/**
 * Hook to trigger hero animations
 * Can be used in page components to coordinate animations
 */
export function useHeroAnimation() {
  const [hasAnimated, setHasAnimated] = useState(false);

  const triggerAnimation = () => {
    setHasAnimated(true);
    setTimeout(() => setHasAnimated(false), 1000);
  };

  return { hasAnimated, triggerAnimation };
}

/**
 * Individual animated wrapper for hero elements
 */
export function HeroItem({
  children,
  delay = 0,
  animation = 'slideUp',
  className = '',
  'aria-hidden': ariaHidden = false
}) {
  const [isVisible, setIsVisible] = useState(false);
  const itemRef = useRef(null);
  const animationsEnabled = areAnimationsEnabled();

  useEffect(() => {
    if (!animationsEnabled) {
      setIsVisible(true);
      return;
    }

    const timer = setTimeout(() => {
      setIsVisible(true);
    }, delay);

    return () => clearTimeout(timer);
  }, [delay, animationsEnabled]);

  return (
    <div
      ref={itemRef}
      className={`hero-item ${animation} ${isVisible ? 'visible' : ''} ${className}`}
      style={{ animationDelay: `${delay}ms` }}
      aria-hidden={ariaHidden || !isVisible}
    >
      {children}
    </div>
  );
}
