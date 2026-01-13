/**
 * Module Theme Provider
 * Feature: 002-ui-improvements
 * Task: T012
 *
 * Applies module-specific theming based on URL path.
 * Wraps the entire Docusaurus application to provide
 * dynamic color theming for different course modules.
 */

import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import { detectModule, applyModuleTheme, MODULE_COLORS } from '../../utils/moduleDetection';
import { initPerformanceMonitoring } from '../../utils/performance';

/**
 * ModuleThemeProvider Component
 *
 * Wraps children and applies module-specific theming
 * based on the current URL path.
 */
export default function ModuleThemeProvider({ children }) {
  const location = useLocation();
  const [currentModule, setCurrentModule] = useState(null);

  useEffect(() => {
    // Initialize performance monitoring on mount
    initPerformanceMonitoring();

    // Detect and apply module theme
    const moduleId = detectModule(location.pathname);
    setCurrentModule(moduleId);
    applyModuleTheme(location.pathname);

    // Listen for reduced motion preference changes
    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
    const handleMotionChange = () => {
      if (mediaQuery.matches) {
        document.body.classList.add('reduced-motion');
      } else {
        document.body.classList.remove('reduced-motion');
      }
    };

    mediaQuery.addEventListener('change', handleMotionChange);

    // Initial check
    handleMotionChange();

    return () => {
      mediaQuery.removeEventListener('change', handleMotionChange);
    };
  }, [location.pathname]);

  // Re-apply theme when module changes
  useEffect(() => {
    if (currentModule) {
      const config = MODULE_COLORS[currentModule];

      // Add transition class for smooth color changes
      document.body.classList.add('theme-transitioning');

      applyModuleTheme(location.pathname);

      // Remove transition class after animation completes
      const timeout = setTimeout(() => {
        document.body.classList.remove('theme-transitioning');
      }, 300);

      return () => clearTimeout(timeout);
    }
  }, [currentModule, location.pathname]);

  return <>{children}</>;
}

/**
 * Hook to use the current module context
 * @returns {Object} Current module info
 */
export function useModuleContext() {
  const location = useLocation();
  const moduleId = detectModule(location.pathname);
  const config = MODULE_COLORS[moduleId];

  return {
    moduleId,
    ...config
  };
}
