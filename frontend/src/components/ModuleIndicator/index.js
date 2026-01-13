/**
 * Module Indicator Component
 * Feature: 002-ui-improvements
 * Task: T033
 *
 * Displays the current course module with visual indicator.
 * Shows module name and color for user orientation.
 */

import React from 'react';
import { useModuleContext } from '../ModuleThemeProvider';
import './styles.css';

/**
 * Module display names and descriptions
 */
const MODULE_INFO = {
  ros2: {
    name: 'ROS 2 Fundamentals',
    shortName: 'ROS 2',
    description: 'Robot Operating System 2'
  },
  digitalTwin: {
    name: 'Digital Twin',
    shortName: 'Digital Twin',
    description: 'Simulation & Virtual Environments'
  },
  isaac: {
    name: 'NVIDIA Isaac',
    shortName: 'Isaac',
    description: 'AI-Powered Robotics Platform'
  },
  vla: {
    name: 'Vision-Language-Action',
    shortName: 'VLA',
    description: 'Conversational Robotics'
  },
  general: {
    name: 'Physical AI',
    shortName: 'General',
    description: 'AI-Native Textbook'
  }
};

/**
 * ModuleIndicator Component
 *
 * Displays a visual indicator showing the current module
 * with module-specific color theming.
 *
 * @param {Object} props
 * @param {boolean} props.compact - Use compact display variant
 * @param {boolean} props.showDescription - Include module description
 * @param {string} props.className - Additional CSS classes
 */
export default function ModuleIndicator({
  compact = false,
  showDescription = false,
  className = ''
}) {
  const { moduleId } = useModuleContext();
  const info = MODULE_INFO[moduleId] || MODULE_INFO.general;

  return (
    <div
      className={`module-indicator module-indicator-${moduleId} ${compact ? 'module-indicator-compact' : ''} ${className}`}
      data-module={moduleId}
      role="banner"
      aria-label={`Current module: ${info.name}`}
    >
      <div className="module-indicator-dot" />
      <div className="module-indicator-content">
        <span className="module-indicator-name">{info.shortName}</span>
        {showDescription && (
          <span className="module-indicator-description">{info.description}</span>
        )}
      </div>
    </div>
  );
}

/**
 * ModuleBadge Component
 *
 * Smaller badge variant for inline module display
 */
export function ModuleBadge({
  moduleId,
  className = ''
}) {
  const info = MODULE_INFO[moduleId] || MODULE_INFO.general;

  return (
    <span
      className={`module-badge module-badge-${moduleId} ${className}`}
      data-module={moduleId}
      aria-label={`Module: ${info.name}`}
    >
      {info.shortName}
    </span>
  );
}

/**
 * ModuleProgress Component
 *
 * Shows current module with progress indicator
 */
export function ModuleProgress({
  currentWeek,
  totalWeeks = 13,
  className = ''
}) {
  const { moduleId } = useModuleContext();
  const info = MODULE_INFO[moduleId] || MODULE_INFO.general;
  const progress = Math.round((currentWeek / totalWeeks) * 100);

  return (
    <div
      className={`module-progress module-progress-${moduleId} ${className}`}
      data-module={moduleId}
      role="progressbar"
      aria-valuenow={progress}
      aria-valuemin={0}
      aria-valuemax={100}
      aria-label={`Course progress: ${progress}%`}
    >
      <div className="module-progress-header">
        <span className="module-progress-name">{info.name}</span>
        <span className="module-progress-percent">{progress}%</span>
      </div>
      <div className="module-progress-bar">
        <div
          className="module-progress-fill"
          style={{ width: `${progress}%` }}
        />
      </div>
      <div className="module-progress-footer">
        <span className="module-progress-current">Week {currentWeek}</span>
        <span className="module-progress-total">of {totalWeeks}</span>
      </div>
    </div>
  );
}
