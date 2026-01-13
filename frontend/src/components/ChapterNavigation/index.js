/**
 * Chapter Navigation Component
 * Feature: 002-ui-improvements
 * Task: T039
 *
 * Provides previous/next navigation between course chapters.
 * Displays chapter titles and module context for orientation.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import { useModuleContext } from '../ModuleThemeProvider';
import './styles.css';

/**
 * Course structure data for navigation
 * Maps weeks to their modules and navigation links
 */
const COURSE_STRUCTURE = [
  // Week 1-2: Introduction (ROS 2)
  { week: 1, title: 'Introduction to Physical AI', path: '/docs/week1/intro-physical-ai', module: 'ros2' },
  { week: 2, title: 'Embodied Intelligence', path: '/docs/week2/embodied-intelligence', module: 'ros2' },

  // Week 3-5: ROS 2 Fundamentals
  { week: 3, title: 'ROS 2 Nodes & Topics', path: '/docs/week3/ros2-nodes-topics', module: 'ros2' },
  { week: 4, title: 'ROS 2 Services & Actions', path: '/docs/week4/ros2-services-actions', module: 'ros2' },
  { week: 5, title: 'ROS 2 Advanced Topics', path: '/docs/week5/ros2-fundamentals-advanced', module: 'ros2' },

  // Week 6-7: Digital Twin
  { week: 6, title: 'Gazebo Simulation', path: '/docs/week6/gazebo-simulation', module: 'digitalTwin' },
  { week: 7, title: 'NVIDIA Isaac Sim', path: '/docs/week7/nvidia-isaac-sim', module: 'digitalTwin' },

  // Week 8-10: Isaac Platform
  { week: 8, title: 'NVIDIA Isaac Sim', path: '/docs/week8/nvidia-isaac-sim', module: 'isaac' },
  { week: 9, title: 'Isaac ROS', path: '/docs/week9/isaac-ros', module: 'isaac' },
  { week: 10, title: 'Sim-to-Real Transfer', path: '/docs/week10/sim-to-real-transfer', module: 'isaac' },

  // Week 11-12: Humanoid Development
  { week: 11, title: 'Humanoid Kinematics', path: '/docs/week11/humanoid-kinematics', module: 'vla' },
  { week: 12, title: 'Humanoid Development', path: '/docs/week12/humanoid-development', module: 'vla' },

  // Week 13: VLA Capstone
  { week: 13, title: 'Conversational Robotics', path: '/docs/week13/conversational-robotics', module: 'vla' }
];

/**
 * Get current chapter index from path
 */
function getCurrentChapterIndex(currentPath) {
  return COURSE_STRUCTURE.findIndex(chapter =>
    currentPath.includes(chapter.path) ||
    currentPath.includes(`/week${chapter.week}/`)
  );
}

/**
 * ChapterNavigation Component
 *
 * Displays previous/next chapter links with module context.
 *
 * @param {Object} props
 * @param {string} props.currentPath - Current page path (auto-detected if not provided)
 * @param {Function} props.onNavigate - Callback when navigation occurs
 */
export default function ChapterNavigation({
  currentPath = '',
  onNavigate
}) {
  const { moduleId } = useModuleContext();

  // Detect current path from URL if not provided
  const [currentIndex, setCurrentIndex] = React.useState(() => {
    if (currentPath) return getCurrentChapterIndex(currentPath);
    if (typeof window !== 'undefined') {
      return getCurrentChapterIndex(window.location.pathname);
    }
    return -1;
  });

  // Update index when path changes
  React.useEffect(() => {
    if (currentPath) {
      setCurrentIndex(getCurrentChapterIndex(currentPath));
    }
  }, [currentPath]);

  const prevChapter = currentIndex > 0 ? COURSE_STRUCTURE[currentIndex - 1] : null;
  const nextChapter = currentIndex >= 0 && currentIndex < COURSE_STRUCTURE.length - 1
    ? COURSE_STRUCTURE[currentIndex + 1]
    : null;

  const handleNavigate = (direction, chapter) => {
    if (onNavigate) {
      onNavigate(direction, chapter);
    }
  };

  // Don't render if we're not on a course page
  if (currentIndex === -1 && !currentPath) {
    return null;
  }

  return (
    <nav
      className="chapter-navigation"
      role="navigation"
      aria-label="Course chapters"
    >
      <div className="chapter-navigation-container">
        {prevChapter && (
          <Link
            to={prevChapter.path}
            className="chapter-nav-link chapter-nav-prev"
            onClick={() => handleNavigate('prev', prevChapter)}
          >
            <div className="chapter-nav-icon">←</div>
            <div className="chapter-nav-content">
              <span className="chapter-nav-label">Previous</span>
              <span className="chapter-nav-title">{prevChapter.title}</span>
              <span className="chapter-nav-meta">Week {prevChapter.week}</span>
            </div>
          </Link>
        )}

        {nextChapter && (
          <Link
            to={nextChapter.path}
            className="chapter-nav-link chapter-nav-next"
            onClick={() => handleNavigate('next', nextChapter)}
          >
            <div className="chapter-nav-content">
              <span className="chapter-nav-label">Next</span>
              <span className="chapter-nav-title">{nextChapter.title}</span>
              <span className="chapter-nav-meta">Week {nextChapter.week}</span>
            </div>
            <div className="chapter-nav-icon">→</div>
          </Link>
        )}
      </div>
    </nav>
  );
}

/**
 * ChapterTOC Component
 *
 * Displays table of contents for the current module
 */
export function ChapterTOC({
  currentWeek,
  className = ''
}) {
  const { moduleId } = useModuleContext();

  const moduleChapters = COURSE_STRUCTURE.filter(ch => ch.module === moduleId);

  if (moduleChapters.length === 0) {
    return null;
  }

  return (
    <nav
      className={`chapter-toc ${className}`}
      role="navigation"
      aria-label={`Chapters in ${moduleId} module`}
    >
      <h3 className="chapter-toc-title">Chapters</h3>
      <ul className="chapter-toc-list">
        {moduleChapters.map((chapter) => (
          <li key={chapter.week} className="chapter-toc-item">
            <Link
              to={chapter.path}
              className={`chapter-toc-link ${currentWeek === chapter.week ? 'chapter-toc-link-active' : ''}`}
              aria-current={currentWeek === chapter.week ? 'location' : undefined}
            >
              <span className="chapter-toc-week">Week {chapter.week}</span>
              <span className="chapter-toc-chapter-title">{chapter.title}</span>
            </Link>
          </li>
        ))}
      </ul>
    </nav>
  );
}

/**
 * Breadcrumb Component
 *
 * Shows hierarchical navigation: Home > Module > Week > Chapter
 */
export function ChapterBreadcrumb({
  currentWeek,
  currentTitle,
  className = ''
}) {
  const { moduleId } = useModuleContext();

  const moduleNames = {
    ros2: 'ROS 2',
    digitalTwin: 'Digital Twin',
    isaac: 'NVIDIA Isaac',
    vla: 'VLA'
  };

  return (
    <nav
      className={`chapter-breadcrumb ${className}`}
      role="navigation"
      aria-label="Breadcrumb navigation"
    >
      <ol className="breadcrumb-list" itemScope itemType="https://schema.org/BreadcrumbList">
        <li className="breadcrumb-item" itemProp="itemListElement" itemScope itemType="https://schema.org/ListItem">
          <Link to="/" className="breadcrumb-link" itemProp="item">
            <span itemProp="name">Home</span>
          </Link>
          <meta itemProp="position" content="1" />
        </li>
        <li className="breadcrumb-item" itemProp="itemListElement" itemScope itemType="https://schema.org/ListItem">
          <Link to="/docs/intro" className="breadcrumb-link" itemProp="item">
            <span itemProp="name">Course</span>
          </Link>
          <meta itemProp="position" content="2" />
        </li>
        {moduleId && (
          <li className="breadcrumb-item" itemProp="itemListElement" itemScope itemType="https://schema.org/ListItem">
            <span className="breadcrumb-current" itemProp="name">
              {moduleNames[moduleId] || 'Module'}
            </span>
            <meta itemProp="position" content="3" />
          </li>
        )}
        {currentWeek && (
          <li className="breadcrumb-item" itemProp="itemListElement" itemScope itemType="https://schema.org/ListItem">
            <span className="breadcrumb-current" itemProp="name">
              Week {currentWeek}
            </span>
            <meta itemProp="position" content={moduleId ? '4' : '3'} />
          </li>
        )}
      </ol>
    </nav>
  );
}

export { COURSE_STRUCTURE };
