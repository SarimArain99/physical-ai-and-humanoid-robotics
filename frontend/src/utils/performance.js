/**
 * Performance Detection Utility
 * Feature: 002-ui-improvements
 * Task: T011
 *
 * Detects device capabilities and adjusts visual effects
 * for optimal performance across all devices
 */

// Performance tier definitions
export const PERFORMANCE_TIERS = {
  LOW: 'low',      // Reduce all effects, minimal animations
  MEDIUM: 'medium', // Moderate effects, reduced particle counts
  HIGH: 'high'      // Full effects, all animations enabled
};

// Configuration for each tier
export const TIER_CONFIG = {
  [PERFORMANCE_TIERS.LOW]: {
    animations: false,
    particles: 0,
    shadows: false,
    blur: false,
    threeD: false,
    scrollAnimations: false
  },
  [PERFORMANCE_TIERS.MEDIUM]: {
    animations: true,
    particles: 50,
    shadows: false,
    blur: false,
    threeD: true,
    scrollAnimations: true
  },
  [PERFORMANCE_TIERS.HIGH]: {
    animations: true,
    particles: 100,
    shadows: true,
    blur: true,
    threeD: true,
    scrollAnimations: true
  }
};

/**
 * Detects the device's performance tier based on hardware capabilities
 * @returns {string} The performance tier (low, medium, high)
 */
export function detectPerformanceTier() {
  if (typeof navigator === 'undefined' || typeof window === 'undefined') {
    return PERFORMANCE_TIERS.MEDIUM; // Default for SSR
  }

  let score = 0;

  // CPU cores (more cores = better performance)
  const cores = navigator.hardwareConcurrency || 2;
  if (cores >= 8) score += 3;
  else if (cores >= 4) score += 2;
  else score += 1;

  // Device memory (more RAM = better performance)
  const memory = (navigator.deviceMemory || 4); // in GB
  if (memory >= 8) score += 3;
  else if (memory >= 4) score += 2;
  else score += 1;

  // Connection type (faster = better performance)
  const connection = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
  if (connection) {
    const effectiveType = connection.effectiveType;
    if (effectiveType === '4g' || !effectiveType) score += 3;
    else if (effectiveType === '3g') score += 2;
    else score += 1; // 2g or slower
  } else {
    score += 2; // Unknown connection, assume moderate
  }

  // Touch device detection (usually less powerful)
  const isTouch = 'ontouchstart' in window || navigator.maxTouchPoints > 0;
  if (isTouch) score -= 1;

  // Determine tier based on score
  if (score >= 8) return PERFORMANCE_TIERS.HIGH;
  if (score >= 5) return PERFORMANCE_TIERS.MEDIUM;
  return PERFORMANCE_TIERS.LOW;
}

/**
 * Gets the configuration for the current device's performance tier
 * @returns {Object} Configuration object with effect settings
 */
export function getPerformanceConfig() {
  const tier = detectPerformanceTier();
  return {
    tier,
    ...TIER_CONFIG[tier]
  };
}

/**
 * Checks if animations should be enabled based on performance and preferences
 * @returns {boolean} True if animations should be enabled
 */
export function areAnimationsEnabled() {
  // Check for reduced motion preference
  if (typeof window !== 'undefined' && window.matchMedia) {
    const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (prefersReducedMotion) {
      return false;
    }
  }

  // Check performance tier
  const config = getPerformanceConfig();
  return config.animations;
}

/**
 * Checks if WebGL is available
 * @returns {boolean} True if WebGL is supported
 */
export function isWebGLAvailable() {
  if (typeof window === 'undefined' || typeof document === 'undefined') {
    return false;
  }

  try {
    const canvas = document.createElement('canvas');
    return !!(
      window.WebGLRenderingContext &&
      (canvas.getContext('webgl') || canvas.getContext('experimental-webgl'))
    );
  } catch (e) {
    return false;
  }
}

/**
 * Checks if WebGL2 is available
 * @returns {boolean} True if WebGL2 is supported
 */
export function isWebGL2Available() {
  if (typeof window === 'undefined' || typeof document === 'undefined') {
    return false;
  }

  try {
    const canvas = document.createElement('canvas');
    return !!(canvas.getContext('webgl2'));
  } catch (e) {
    return false;
  }
}

/**
 * Gets the best available WebGL context
 * @returns {string|null} 'webgl2', 'webgl', or null
 */
export function getWebGLContext() {
  if (isWebGL2Available()) return 'webgl2';
  if (isWebGLAvailable()) return 'webgl';
  return null;
}

/**
 * Checks if the device is mobile
 * @returns {boolean} True if the device is mobile
 */
export function isMobile() {
  if (typeof window === 'undefined') return false;

  const userAgent = navigator.userAgent || navigator.vendor || window.opera;
  return /android|iphone|ipad|ipod|blackberry|iemobile|opera mini/i.test(userAgent);
}

/**
 * Gets the appropriate particle count based on performance
 * @param {number} defaultCount - The default particle count
 * @returns {number} The adjusted particle count
 */
export function getParticleCount(defaultCount = 100) {
  const config = getPerformanceConfig();
  return Math.min(defaultCount, config.particles);
}

/**
 * Debounce function for performance optimization
 * @param {Function} func - The function to debounce
 * @param {number} wait - The wait time in milliseconds
 * @returns {Function} The debounced function
 */
export function debounce(func, wait = 100) {
  let timeout;
  return function executedFunction(...args) {
    const later = () => {
      clearTimeout(timeout);
      func(...args);
    };
    clearTimeout(timeout);
    timeout = setTimeout(later, wait);
  };
}

/**
 * Throttle function for performance optimization
 * @param {Function} func - The function to throttle
 * @param {number} limit - The time limit in milliseconds
 * @returns {Function} The throttled function
 */
export function throttle(func, limit = 100) {
  let inThrottle;
  return function executedFunction(...args) {
    if (!inThrottle) {
      func(...args);
      inThrottle = true;
      setTimeout(() => inThrottle = false, limit);
    }
  };
}

/**
 * Request animation frame with fallback
 * @param {Function} callback - The callback function
 * @returns {number} The request ID
 */
export function requestAnimationFrame(callback) {
  if (typeof window !== 'undefined' && window.requestAnimationFrame) {
    return window.requestAnimationFrame(callback);
  }
  // Fallback for older browsers
  return setTimeout(callback, 16);
}

/**
 * Cancel animation frame with fallback
 * @param {number} requestId - The request ID
 */
export function cancelAnimationFrame(requestId) {
  if (typeof window !== 'undefined' && window.cancelAnimationFrame) {
    return window.cancelAnimationFrame(requestId);
  }
  // Fallback for older browsers
  return clearTimeout(requestId);
}

/**
 * React hook for performance detection
 * @returns {Object} Performance information and utilities
 */
export function usePerformance() {
  return {
    tier: detectPerformanceTier(),
    config: getPerformanceConfig(),
    animationsEnabled: areAnimationsEnabled(),
    webglAvailable: isWebGLAvailable(),
    webgl2Available: isWebGL2Available(),
    isMobile: isMobile(),
    getParticleCount,
    debounce,
    throttle,
    requestAnimationFrame,
    cancelAnimationFrame
  };
}

/**
 * Initialize performance monitoring
 * Adds data attributes to body for CSS targeting
 */
export function initPerformanceMonitoring() {
  if (typeof document === 'undefined') return;

  const tier = detectPerformanceTier();
  const animationsEnabled = areAnimationsEnabled();

  document.body.setAttribute('data-performance-tier', tier);
  document.body.setAttribute('data-animations', animationsEnabled ? 'enabled' : 'disabled');

  if (!animationsEnabled) {
    document.body.classList.add('reduced-motion');
  }
}

export default {
  PERFORMANCE_TIERS,
  TIER_CONFIG,
  detectPerformanceTier,
  getPerformanceConfig,
  areAnimationsEnabled,
  isWebGLAvailable,
  isWebGL2Available,
  getWebGLContext,
  isMobile,
  getParticleCount,
  debounce,
  throttle,
  requestAnimationFrame,
  cancelAnimationFrame,
  usePerformance,
  initPerformanceMonitoring
};
