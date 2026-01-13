/**
 * Chat Icon Component
 * Feature: 002-ui-improvements
 * Task: T023
 *
 * Custom SVG robot icon for the chat toggle button.
 * Replaces the emoji-based icon with a professional illustration.
 */

import React from 'react';
import './ChatIcon.css';

/**
 * ChatIcon Component
 *
 * Custom SVG robot icon with optional animations.
 *
 * @param {Object} props
 * @param {boolean} props.isOpen - Whether the chat is currently open
 * @param {number} props.unreadCount - Number of unread messages (for badge)
 * @param {boolean} props.pulse - Whether to show pulsing animation
 * @param {number} props.size - Icon size in pixels (default: 24)
 * @param {Function} props.onClick - Click handler
 * @param {string} props.className - Additional CSS classes
 * @param {string} props.color - Icon color (default: currentColor)
 */
export default function ChatIcon({
  isOpen = false,
  unreadCount = 0,
  pulse = true,
  size = 24,
  onClick,
  className = '',
  color = 'currentColor'
}) {
  return (
    <div
      className={`chat-icon-container ${isOpen ? 'chat-icon-open' : ''} ${pulse ? 'chat-icon-pulse' : ''} ${className}`}
      onClick={onClick}
      role="button"
      tabIndex={0}
      aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
      aria-expanded={isOpen}
      style={{ width: size, height: size }}
    >
      <svg
        width={size}
        height={size}
        viewBox="0 0 24 24"
        fill="none"
        stroke={color}
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
        className="chat-icon-svg"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Robot head outline */}
        <rect x="6" y="4" width="12" height="14" rx="2" className="robot-head" />

        {/* Eyes */}
        <circle cx="9" cy="10" r="1.5" fill={color} className="robot-eye robot-eye-left" />
        <circle cx="15" cy="10" r="1.5" fill={color} className="robot-eye robot-eye-right" />

        {/* Antenna */}
        <line x1="12" y1="4" x2="12" y2="1" className="robot-antenna" />
        <circle cx="12" cy="1" r="1" fill={color} className="robot-antenna-tip" />

        {/* Mouth - changes based on open state */}
        {isOpen ? (
          <path d="M 8 14 Q 12 17 16 14" className="robot-mouth robot-mouth-smile" />
        ) : (
          <line x1="9" y1="14" x2="15" y2="14" className="robot-mouth robot-mouth-neutral" />
        )}
      </svg>

      {/* Unread badge */}
      {unreadCount > 0 && (
        <span className="chat-icon-badge" aria-label={`${unreadCount} unread messages`}>
          {unreadCount > 9 ? '9+' : unreadCount}
        </span>
      )}

      {/* Pulsing glow effect */}
      {pulse && <span className="chat-icon-glow" />}
    </div>
  );
}

/**
 * ChatIconWithAnimation Component
 * Includes built-in animation on mount
 */
export function ChatIconWithAnimation(props) {
  const [isMounted, setIsMounted] = React.useState(false);

  React.useEffect(() => {
    setIsMounted(true);
  }, []);

  return <ChatIcon {...props} className={`${props.className || ''} ${isMounted ? 'chat-icon-mounted' : ''}`} />;
}

/**
 * MinimalChatIcon Component
 * Smaller, simpler version for tight spaces
 */
export function MinimalChatIcon({
  isOpen = false,
  size = 20,
  onClick,
  className = ''
}) {
  return (
    <div
      className={`minimal-chat-icon ${isOpen ? 'minimal-chat-icon-open' : ''} ${className}`}
      onClick={onClick}
      role="button"
      tabIndex={0}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      style={{ width: size, height: size }}
    >
      <svg
        width={size}
        height={size}
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        {/* Simplified robot head */}
        <rect x="7" y="5" width="10" height="12" rx="2" />
        <circle cx="10" cy="10" r="1" fill="currentColor" />
        <circle cx="14" cy="10" r="1" fill="currentColor" />
        <line x1="12" y1="5" x2="12" y2="3" />
        <circle cx="12" cy="3" r="0.5" fill="currentColor" />
        {isOpen ? (
          <path d="M 9 13 Q 12 15 15 13" />
        ) : (
          <line x1="10" y1="13" x2="14" y2="13" />
        )}
      </svg>
    </div>
  );
}
