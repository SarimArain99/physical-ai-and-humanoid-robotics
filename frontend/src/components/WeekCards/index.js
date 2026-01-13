/**
 * Week Cards Component
 * Feature: 002-ui-improvements
 * Task: T017
 *
 * Enhanced week cards with hover effects and module-specific theming.
 * Displays course weeks as interactive cards with visual feedback.
 */

import React from 'react';
import { useModuleContext } from '../ModuleThemeProvider';
import './styles.css';

/**
 * Week data structure
 * @typedef {Object} WeekData
 * @property {number} week - Week number
 * @property {string} title - Week title
 * @property {string} description - Brief description
 * @property {string} module - Module ID (ros2, digitalTwin, isaac, vla)
 * @property {string} link - Link to week content
 */

/**
 * Individual Week Card Component
 */
export function WeekCard({
  week,
  title,
  description,
  module,
  link,
  completed = false
}) {
  const { moduleId } = useModuleContext();

  return (
    <a
      href={link}
      className={`week-card week-card-${module} ${completed ? 'week-card-completed' : ''}`}
      data-module={module}
    >
      <div className="week-card-header">
        <span className="week-card-number">Week {week}</span>
        {completed && <span className="week-card-badge">✓</span>}
      </div>
      <h3 className="week-card-title">{title}</h3>
      <p className="week-card-description">{description}</p>
      <div className="week-card-footer">
        <span className="week-card-module">{getModuleName(module)}</span>
        <span className="week-card-arrow">→</span>
      </div>
    </a>
  );
}

/**
 * Week Cards Grid Component
 * Displays all week cards in a responsive grid
 */
export default function WeekCards({ weeks = [] }) {
  // Default week data if not provided
  const defaultWeeks = [
    {
      week: 1,
      title: 'Introduction to Physical AI',
      description: 'Understanding the bridge between digital AI and physical robots',
      module: 'ros2',
      link: '/docs/week1/intro-physical-ai'
    },
    {
      week: 2,
      title: 'ROS 2 Fundamentals',
      description: 'Core concepts of Robot Operating System 2',
      module: 'ros2',
      link: '/docs/week2/ros2-fundamentals'
    },
    {
      week: 3,
      title: 'ROS 2 Communication',
      description: 'Nodes, topics, services, and actions',
      module: 'ros2',
      link: '/docs/week3/ros2-communication'
    },
    {
      week: 4,
      title: 'ROS 2 Tools & Best Practices',
      description: 'Launch files, parameters, and debugging',
      module: 'ros2',
      link: '/docs/week4/ros2-tools'
    },
    {
      week: 5,
      title: 'ROS 2 Project',
      description: 'Build your first ROS 2 application',
      module: 'ros2',
      link: '/docs/week5/ros2-project'
    },
    {
      week: 6,
      title: 'Digital Twin Introduction',
      description: 'Creating virtual representations of physical systems',
      module: 'digitalTwin',
      link: '/docs/week6/digital-twin-intro'
    },
    {
      week: 7,
      title: 'Simulation & Testing',
      description: 'Testing robots in virtual environments',
      module: 'digitalTwin',
      link: '/docs/week7/simulation-testing'
    },
    {
      week: 8,
      title: 'NVIDIA Isaac SDK',
      description: 'AI platform for robotics and manipulation',
      module: 'isaac',
      link: '/docs/week8/isaac-intro'
    },
    {
      week: 9,
      title: 'Isaac Sim',
      description: 'Physics-based simulation for robotics',
      module: 'isaac',
      link: '/docs/week9/isaac-sim'
    },
    {
      week: 10,
      title: 'Isaac Navigation',
      description: 'Autonomous navigation with NVIDIA Isaac',
      module: 'isaac',
      link: '/docs/week10/isaac-navigation'
    },
    {
      week: 11,
      title: 'Vision-Language Models',
      description: 'Introduction to VLA for robotics',
      module: 'vla',
      link: '/docs/week11/vla-intro'
    },
    {
      week: 12,
      title: 'VLA Integration',
      description: 'Integrating VLA with robotic systems',
      module: 'vla',
      link: '/docs/week12/vla-integration'
    },
    {
      week: 13,
      title: 'Capstone Project',
      description: 'Build a complete VLA-powered robot',
      module: 'vla',
      link: '/docs/week13/capstone'
    }
  ];

  const weeksData = weeks.length > 0 ? weeks : defaultWeeks;

  return (
    <div className="week-cards-container">
      <div className="week-cards-grid">
        {weeksData.map((weekData) => (
          <WeekCard key={weekData.week} {...weekData} />
        ))}
      </div>
    </div>
  );
}

/**
 * Get display name for module
 */
function getModuleName(module) {
  const names = {
    ros2: 'ROS 2',
    digitalTwin: 'Digital Twin',
    isaac: 'NVIDIA Isaac',
    vla: 'VLA'
  };
  return names[module] || 'General';
}
