/**
 * Week Cards Component
 *
 * One source of truth for the 13 chapter cards on the homepage.
 * Every link below must match a real doc slug (see sidebars.ts) —
 * stale slugs shipped 12 dead 404 cards once before, so keep this
 * list in sync with the docs folder. Run `npm run build` with
 * onBrokenLinks: "throw" to catch any drift.
 */

import React from 'react';
import { useModuleContext } from '../ModuleThemeProvider';
import './styles.css';

/**
 * Week data structure
 * @typedef {Object} WeekData
 * @property {number} week - Week number
 * @property {string} title - Week title (matches the chapter's H1)
 * @property {string} description - Brief description
 * @property {string} module - Module ID (intro, ros2, digitalTwin, isaac, humanoid, vla)
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
  // The 13 real chapters, in order. Slugs and titles match the docs.
  const defaultWeeks = [
    {
      week: 1,
      title: 'Introduction to Physical AI',
      description: 'What physical AI is, and how digital intelligence meets real robots',
      module: 'intro',
      link: '/docs/week1/intro-physical-ai'
    },
    {
      week: 2,
      title: 'Embodied Intelligence',
      description: 'How robots sense, perceive, and act in the physical world',
      module: 'intro',
      link: '/docs/week2/embodied-intelligence'
    },
    {
      week: 3,
      title: 'ROS 2 Architecture and Core Concepts: Nodes and Topics',
      description: 'The building blocks of ROS 2 communication',
      module: 'ros2',
      link: '/docs/week3/ros2-nodes-topics'
    },
    {
      week: 4,
      title: 'ROS 2 Services and Actions',
      description: 'Request-response calls and long-running tasks in ROS 2',
      module: 'ros2',
      link: '/docs/week4/ros2-services-actions'
    },
    {
      week: 5,
      title: 'Advanced ROS 2 Concepts: Launch Files, Parameters, and Lifecycle Nodes',
      description: 'Starting, configuring, and managing ROS 2 systems',
      module: 'ros2',
      link: '/docs/week5/ros2-fundamentals-advanced'
    },
    {
      week: 6,
      title: 'Robot Simulation with Gazebo',
      description: 'Build and test robots in the Gazebo simulator',
      module: 'digitalTwin',
      link: '/docs/week6/gazebo-simulation'
    },
    {
      week: 7,
      title: 'NVIDIA Isaac Sim Fundamentals',
      description: 'First steps with NVIDIA Isaac Sim',
      module: 'digitalTwin',
      link: '/docs/week7/nvidia-isaac-sim'
    },
    {
      week: 8,
      title: 'NVIDIA Isaac Platform: AI-Powered Robotics',
      description: 'The Isaac platform and AI-powered robotics',
      module: 'isaac',
      link: '/docs/week8/nvidia-isaac-sim'
    },
    {
      week: 9,
      title: 'Isaac ROS: Hardware-Accelerated Perception and Navigation',
      description: 'Perception and navigation with Isaac ROS',
      module: 'isaac',
      link: '/docs/week9/isaac-ros'
    },
    {
      week: 10,
      title: 'Sim-to-Real Transfer: Bridging Simulation and Reality',
      description: 'Moving a robot from simulation to the real world',
      module: 'isaac',
      link: '/docs/week10/sim-to-real-transfer'
    },
    {
      week: 11,
      title: 'Humanoid Robot Kinematics',
      description: 'How humanoid robots move: joints, frames, and poses',
      module: 'humanoid',
      link: '/docs/week11/humanoid-kinematics'
    },
    {
      week: 12,
      title: 'Humanoid Robot Development: Kinematics, Dynamics, and Control',
      description: 'Dynamics and control for humanoid robots',
      module: 'humanoid',
      link: '/docs/week12/humanoid-development'
    },
    {
      week: 13,
      title: 'Conversational Robotics and Capstone Integration',
      description: 'Talking robots and the final capstone project',
      module: 'vla',
      link: '/docs/week13/conversational-robotics'
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
    intro: 'Introduction',
    ros2: 'ROS 2',
    digitalTwin: 'Simulation',
    isaac: 'NVIDIA Isaac',
    humanoid: 'Humanoid',
    vla: 'Conversational'
  };
  return names[module] || 'General';
}
