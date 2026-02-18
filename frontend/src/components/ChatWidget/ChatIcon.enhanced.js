/**
 * Enhanced Chat Icon Component
 * Feature: 002-ui-improvements
 * Task: Improved chatbot icon design
 *
 * Modern, visually appealing chat toggle button with gradient background
 * and smooth animations.
 */

import React from 'react';
import './ChatIcon.enhanced.css';

/**
 * EnhancedChatIcon Component
 *
 * Modern chat icon with gradient background and glow effects.
 *
 * @param {Object} props
 * @param {boolean} props.isOpen - Whether the chat is currently open
 * @param {number} props.unreadCount - Number of unread messages (for badge)
 * @param {boolean} props.pulse - Whether to show pulsing animation
 * @param {number} props.size - Icon size in pixels (default: 60)
 * @param {Function} props.onClick - Click handler
 * @param {string} props.className - Additional CSS classes
 */
export default function EnhancedChatIcon({
  isOpen = false,
  unreadCount = 0,
  pulse = true,
  size = 60,
  onClick,
  className = ''
}) {
  return (
    <div
      className={`enhanced-chat-icon ${isOpen ? 'enhanced-chat-icon--open' : ''} ${pulse ? 'enhanced-chat-icon--pulse' : ''} ${className}`}
      onClick={onClick}
      role="button"
      tabIndex={0}
      aria-label={isOpen ? 'Close chat' : 'Open AI assistant'}
      aria-expanded={isOpen}
      style={{ width: size, height: size }}
    >
      {/* Glow effect behind */}
      <span className="enhanced-chat-icon__glow"></span>

      {/* Gradient background */}
      <span className="enhanced-chat-icon__bg"></span>

      {/* Robot icon */}
      <svg
        className="enhanced-chat-icon__svg"
        width={size * 0.5}
        height={size * 0.5}
        viewBox="0 0 24 24"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Robot head */}
        <path
          className="enhanced-chat-icon__head"
          d="M6 7C6 5.34315 7.34315 4 9 4H15C16.6569 4 18 5.34315 18 7V17C18 18.6569 16.6569 20 15 20H9C7.34315 20 6 18.6569 6 17V7Z"
          fill="currentColor"
        />

        {/* Face screen */}
        <rect
          className="enhanced-chat-icon__screen"
          x="8"
          y="8"
          width="8"
          height="8"
          rx="2"
          fill="#1E2A38"
        />

        {/* Eyes */}
        <circle
          className="enhanced-chat-icon__eye enhanced-chat-icon__eye--left"
          cx="10"
          cy="11"
          r="1.5"
          fill="var(--chat-accent, #2ECC71)"
        />
        <circle
          className="enhanced-chat-icon__eye enhanced-chat-icon__eye--right"
          cx="14"
          cy="11"
          r="1.5"
          fill="var(--chat-accent, #2ECC71)"
        />

        {/* Antenna */}
        <path
          className="enhanced-chat-icon__antenna"
          d="M12 4V2"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
        />
        <circle
          className="enhanced-chat-icon__antenna-tip"
          cx="12"
          cy="1.5"
          r="1"
          fill="var(--chat-accent, #2ECC71)"
        />

        {/* Mouth - changes based on state */}
        {isOpen ? (
          <path
            className="enhanced-chat-icon__mouth enhanced-chat-icon__mouth--smile"
            d="M9 15C10 16 14 16 15 15"
            stroke="var(--chat-accent, #2ECC71)"
            strokeWidth="1.5"
            strokeLinecap="round"
          />
        ) : (
          <line
            className="enhanced-chat-icon__mouth enhanced-chat-icon__mouth--neutral"
            x1="10"
            y1="15"
            x2="14"
            y2="15"
            stroke="var(--chat-accent, #2ECC71)"
            strokeWidth="1.5"
            strokeLinecap="round"
          />
        )}
      </svg>

      {/* Unread badge */}
      {unreadCount > 0 && (
        <span className="enhanced-chat-icon__badge" aria-label={`${unreadCount} unread messages`}>
          {unreadCount > 9 ? '9+' : unreadCount}
        </span>
      )}

      {/* Rings animation */}
      {pulse && <span className="enhanced-chat-icon__ring enhanced-chat-icon__ring--1"></span>}
      {pulse && <span className="enhanced-chat-icon__ring enhanced-chat-icon__ring--2"></span>}
    </div>
  );
}

/**
 * CompactChatIcon - Smaller version for tight spaces
 */
export function CompactChatIcon({
  isOpen = false,
  size = 48,
  onClick,
  className = ''
}) {
  return (
    <div
      className={`compact-chat-icon ${isOpen ? 'compact-chat-icon--open' : ''} ${className}`}
      onClick={onClick}
      role="button"
      tabIndex={0}
      aria-label={isOpen ? 'Close chat' : 'Open AI assistant'}
      style={{ width: size, height: size }}
    >
      <svg
        width={size * 0.55}
        height={size * 0.55}
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <rect x="7" y="5" width="10" height="12" rx="2" />
        <circle cx="10" cy="10" r="0.5" fill="currentColor" />
        <circle cx="14" cy="10" r="0.5" fill="currentColor" />
        <line x1="12" y1="5" x2="12" y2="3" />
        <circle cx="12" cy="2.5" r="0.5" fill="currentColor" />
        {isOpen ? (
          <path d="M9 13.5C10 14.5 14 14.5 15 13.5" />
        ) : (
          <line x1="10" y1="13.5" x2="14" y2="13.5" />
        )}
      </svg>
    </div>
  );
}
