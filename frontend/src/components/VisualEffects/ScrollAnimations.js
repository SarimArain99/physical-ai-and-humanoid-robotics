import React, { useRef, useEffect, useCallback } from 'react';
import './styles.css';

/**
 * Scroll Animations Component
 * Feature: 002-ui-improvements
 * Task: T081, T084, T087
 *
 * Intersection Observer-based scroll animations with:
 * - Fade in on scroll
 * - Slide up effects
 * - Staggered children animations
 * - Performance optimization
 */

export function ScrollAnimations({
  rootMargin = '0px',
  threshold = 0.1,
  triggerOnce = true
}) {
  const observerRef = useRef(null);

  useEffect(() => {
    // Check for reduced motion
    const prefersReducedMotion = window.matchMedia &&
      window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    if (prefersReducedMotion) {
      return;
    }

    // Create Intersection Observer
    observerRef.current = new IntersectionObserver(
      (entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            entry.target.classList.add('visible');
            entry.target.classList.remove('hidden');

            // Get stagger index and apply delay
            const staggerIndex = parseInt(entry.target.dataset.staggerIndex || '0');
            if (staggerIndex > 0) {
              entry.target.style.transitionDelay = `${staggerIndex * 100}ms`;
            }

            if (triggerOnce) {
              observerRef.current.unobserve(entry.target);
            }
          } else if (!triggerOnce) {
            entry.target.classList.remove('visible');
            entry.target.classList.add('hidden');
          }
        });
      },
      {
        rootMargin,
        threshold
      }
    );

    // Observe all elements with data-animate attribute
    const animatedElements = document.querySelectorAll('[data-animate]');
    animatedElements.forEach(el => {
      el.classList.add('hidden');
      observerRef.current.observe(el);
    });

    return () => {
      if (observerRef.current) {
        observerRef.current.disconnect();
      }
    };
  }, [rootMargin, threshold, triggerOnce]);

  // This component doesn't render anything
  return null;
}

/**
 * Hook to add scroll animation to elements
 */
export function useScrollAnimation(options = {}) {
  const ref = useRef(null);

  useEffect(() => {
    const element = ref.current;
    if (!element) return;

    // Check for reduced motion
    const prefersReducedMotion = window.matchMedia &&
      window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    if (prefersReducedMotion) {
      element.classList.add('visible');
      return;
    }

    element.classList.add('hidden');
    element.setAttribute('data-animate', 'true');

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            entry.target.classList.add('visible');
            entry.target.classList.remove('hidden');

            if (options.triggerOnce !== false) {
              observer.unobserve(entry.target);
            }
          } else if (options.triggerOnce === false) {
            entry.target.classList.remove('visible');
            entry.target.classList.add('hidden');
          }
        });
      },
      {
        threshold: options.threshold || 0.1,
        rootMargin: options.rootMargin || '0px'
      }
    );

    observer.observe(element);

    return () => observer.disconnect();
  }, [options]);

  return ref;
}

/**
 * Animated wrapper component
 */
export function Animated({
  children,
  animation = 'fade-up',
  delay = 0,
  staggerIndex = 0,
  className = '',
  ...props
}) {
  const ref = useScrollAnimation();

  const animationClass = animation || 'fade-up';

  return (
    <div
      ref={ref}
      className={`animated ${animationClass} ${className}`}
      data-stagger-index={staggerIndex}
      style={{ transitionDelay: delay > 0 ? `${delay}ms` : undefined }}
      {...props}
    >
      {children}
    </div>
  );
}

export default ScrollAnimations;
