/**
 * Module Detection Utility
 * Feature: 002-ui-improvements
 * Task: T010
 *
 * Detects the current course module based on URL path
 * and provides module-specific configuration
 */

// Module color configurations — one green family only (locked palette).
// Values are fills and dots, never body text, so the brighter greens
// below read on both light and dark surfaces.
export const MODULE_COLORS = {
  ros2: {
    primary: '#34D399',
    accent: '#6EE7B7',
    light: '#D1FAE5',
    dark: '#059669',
    name: 'ROS 2 Fundamentals'
  },
  digitalTwin: {
    primary: '#4ADE80',
    accent: '#86EFAC',
    light: '#DCFCE7',
    dark: '#16A34A',
    name: 'Digital Twin'
  },
  isaac: {
    primary: '#27AD72',
    accent: '#4ADE80',
    light: '#A3FAD9',
    dark: '#1F9260',
    name: 'NVIDIA Isaac'
  },
  vla: {
    primary: '#1F9260',
    accent: '#34D399',
    light: '#A3FAD9',
    dark: '#177245',
    name: 'Vision-Language-Action'
  },
  default: {
    primary: '#2ECC71',
    accent: '#22C55E',
    light: '#A3FAD9',
    dark: '#25B365',
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
