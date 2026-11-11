/**
 * Module Detection Utility
 * Feature: 002-ui-improvements
 * Task: T010
 *
 * Detects the current course module based on URL path
 * and provides module-specific configuration
 */

// Module color configurations — one blue family only (locked palette).
// Values are fills and dots, never body text, so the brighter greens
// below read on both light and dark surfaces.
export const MODULE_COLORS = {
  ros2: {
    primary: '#58A6FF',
    accent: '#79B8FF',
    light: '#D2E9FF',
    dark: '#1F6FEB',
    name: 'ROS 2 Fundamentals'
  },
  digitalTwin: {
    primary: '#79B8FF',
    accent: '#A5D6FF',
    light: '#EAF4FF',
    dark: '#388BFD',
    name: 'Digital Twin'
  },
  isaac: {
    primary: '#1F6FEB',
    accent: '#58A6FF',
    light: '#D2E9FF',
    dark: '#0969DA',
    name: 'NVIDIA Isaac'
  },
  vla: {
    primary: '#388BFD',
    accent: '#79B8FF',
    light: '#D2E9FF',
    dark: '#0550AE',
    name: 'Vision-Language-Action'
  },
  default: {
    primary: '#58A6FF',
    accent: '#79B8FF',
    light: '#D2E9FF',
    dark: '#1F6FEB',
    name: 'Physical AI'
  }
};

/**
 * Detects the current module based on URL pathname
 * @param {string} pathname - The current URL pathname
 * @returns {string} The module ID (ros2, digitalTwin, isaac, vla, or default)
 */
export function detectModule(pathname) {
  // Normalize the path
  const path = pathname.toLowerCase();

  // ROS 2 Module: Intro through Week 5
  if (/^\/(docs\/)?(intro|week[1-5]|introduction|getting-started)/.test(path)) {
    return 'ros2';
  }

  // Digital Twin Module: Week 6-7
  if (/^\/(docs\/)?week[6-7]/.test(path)) {
    return 'digitalTwin';
  }

  // NVIDIA Isaac Module: Week 8-10
  if (/^\/(docs\/)?week[8-9]|week10/.test(path)) {
    return 'isaac';
  }

  // VLA Module: Week 11-13
  if (/^\/(docs\/)?week(11|12|13)/.test(path)) {
    return 'vla';
  }

  // Default module
  return 'default';
}

/**
 * Gets the module configuration for the current path
 * @param {string} pathname - The current URL pathname
 * @returns {Object} Module configuration object with colors and name
 */
export function getModuleConfig(pathname) {
  const moduleId = detectModule(pathname);
  return MODULE_COLORS[moduleId];
}

/**
 * Applies module theming to the document body
 * @param {string} pathname - The current URL pathname
 */
export function applyModuleTheme(pathname) {
  const moduleId = detectModule(pathname);
  const config = MODULE_COLORS[moduleId];

  // Set data-module attribute on body
  if (typeof document !== 'undefined') {
    document.body.setAttribute('data-module', moduleId);

    // Set CSS custom properties for module colors
    const root = document.documentElement;
    root.style.setProperty('--module-primary', config.primary);
    root.style.setProperty('--module-accent', config.accent);
    root.style.setProperty('--module-light', config.light);
    root.style.setProperty('--module-dark', config.dark);
  }
}

/**
 * Hook for React components to use module detection
 * Works with Docusaurus useLocation hook
 * @returns {Object} { moduleId, config, applyTheme }
 */
export function useModule(pathname) {
  const moduleId = detectModule(pathname);
  const config = MODULE_COLORS[moduleId];

  return {
    moduleId,
    config,
    applyTheme: () => applyModuleTheme(pathname)
  };
}

/**
 * Gets all available modules
 * @returns {Array} Array of module objects with id and name
 */
export function getAllModules() {
  return Object.entries(MODULE_COLORS).map(([id, config]) => ({
    id,
    name: config.name,
    primary: config.primary
  }));
}

/**
 * Checks if a path belongs to a specific module
 * @param {string} pathname - The path to check
 * @param {string} moduleId - The module ID to check against
 * @returns {boolean} True if the path belongs to the module
 */
export function isModulePath(pathname, moduleId) {
  return detectModule(pathname) === moduleId;
}

export default {
  detectModule,
  getModuleConfig,
  applyModuleTheme,
  useModule,
  getAllModules,
  isModulePath,
  MODULE_COLORS
};
