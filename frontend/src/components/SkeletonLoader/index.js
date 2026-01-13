/**
 * Skeleton Loader Component
 * Feature: 002-ui-improvements
 * Task: T047
 *
 * Provides loading placeholder animations while content is being fetched.
 * Supports various skeleton types: text, card, avatar, and custom.
 */

import React from 'react';
import { areAnimationsEnabled } from '../../utils/performance';
import './styles.css';

/**
 * SkeletonLoader Component
 *
 * Displays a pulsing placeholder while content loads.
 *
 * @param {Object} props
 * @param {string} props.variant - Skeleton type: 'text', 'circle', 'rect', 'card'
 * @param {number} props.width - Width in pixels or percentage
 * @param {number} props.height - Height in pixels
 * @param {boolean} props.disabled - Disable shimmer animation
 * @param {string} props.className - Additional CSS classes
 * @param {number} props.lines - Number of text lines (for text variant)
 * @param {boolean} props.avatar - Show avatar placeholder
 */
export default function SkeletonLoader({
  variant = 'text',
  width,
  height,
  disabled = false,
  className = '',
  lines = 1,
  avatar = false
}) {
  const animationsEnabled = !disabled && areAnimationsEnabled();

  const style = {};
  if (width) style.width = typeof width === 'number' ? `${width}px` : width;
  if (height) style.height = typeof height === 'number' ? `${height}px` : height;

  if (variant === 'card') {
    return <CardSkeleton disabled={disabled} className={className} />;
  }

  if (variant === 'chat') {
    return <ChatSkeleton disabled={disabled} className={className} />;
  }

  if (lines > 1) {
    return (
      <div className={`skeleton-lines ${className}`}>
        {avatar && <CircleSkeleton width={40} height={40} disabled={disabled} />}
        <div className="skeleton-lines-content">
          {Array.from({ length: lines }).map((_, i) => (
            <div
              key={i}
              className={`skeleton skeleton-text ${animationsEnabled ? 'skeleton-animate' : ''}`}
              style={{
                ...style,
                width: i === lines - 1 ? '60%' : undefined
              }}
            />
          ))}
        </div>
      </div>
    );
  }

  const variantClass = `skeleton-${variant}`;

  return (
    <div
      className={`skeleton ${variantClass} ${animationsEnabled ? 'skeleton-animate' : ''} ${className}`}
      style={style}
      aria-hidden="true"
    />
  );
}

/**
 * Text Skeleton - Simulates text lines
 */
export function TextSkeleton({ lines = 3, className = '' }) {
  return (
    <div className={`skeleton-text-block ${className}`}>
      {Array.from({ length: lines }).map((_, i) => (
        <div
          key={i}
          className="skeleton skeleton-text skeleton-animate"
          style={{ width: i === lines - 1 ? '70%' : '100%' }}
        />
      ))}
    </div>
  );
}

/**
 * Circle Skeleton - For avatars and icons
 */
export function CircleSkeleton({ size = 40, className = '', disabled = false }) {
  const animationsEnabled = !disabled && areAnimationsEnabled();

  return (
    <div
      className={`skeleton skeleton-circle ${animationsEnabled ? 'skeleton-animate' : ''} ${className}`}
      style={{ width: size, height: size }}
      aria-hidden="true"
    />
  );
}

/**
 * Rect Skeleton - For images and cards
 */
export function RectSkeleton({ width = '100%', height = 100, className = '', disabled = false }) {
  const animationsEnabled = !disabled && areAnimationsEnabled();

  return (
    <div
      className={`skeleton skeleton-rect ${animationsEnabled ? 'skeleton-animate' : ''} ${className}`}
      style={{ width, height }}
      aria-hidden="true"
    />
  );
}

/**
 * Card Skeleton - Complete card placeholder
 */
function CardSkeleton({ disabled = false, className = '' }) {
  const animationsEnabled = !disabled && areAnimationsEnabled();

  return (
    <div className={`skeleton-card ${className}`} role="status" aria-label="Loading content">
      <div className={`skeleton skeleton-card-image ${animationsEnabled ? 'skeleton-animate' : ''}`} />
      <div className="skeleton-card-content">
        <div className={`skeleton skeleton-card-title ${animationsEnabled ? 'skeleton-animate' : ''}`} />
        <div className={`skeleton skeleton-card-text ${animationsEnabled ? 'skeleton-animate' : ''}`} />
        <div className={`skeleton skeleton-card-text ${animationsEnabled ? 'skeleton-animate' : ''}`} style={{ width: '60%' }} />
      </div>
    </div>
  );
}

/**
 * Chat Skeleton - For chat message placeholders
 */
function ChatSkeleton({ disabled = false, className = '' }) {
  const animationsEnabled = !disabled && areAnimationsEnabled();

  return (
    <div className={`skeleton-chat ${className}`}>
      <div className="skeleton-chat-avatar">
        <CircleSkeleton size={32} disabled={disabled} />
      </div>
      <div className="skeleton-chat-bubble">
        <div className={`skeleton skeleton-chat-line-1 ${animationsEnabled ? 'skeleton-animate' : ''}`} />
        <div className={`skeleton skeleton-chat-line-2 ${animationsEnabled ? 'skeleton-animate' : ''}`} />
        <div className={`skeleton skeleton-chat-line-3 ${animationsEnabled ? 'skeleton-animate' : ''}`} style={{ width: '40%' }} />
      </div>
    </div>
  );
}

/**
 * Table Skeleton - For data table placeholders
 */
export function TableSkeleton({ rows = 5, columns = 4, className = '' }) {
  return (
    <div className={`skeleton-table ${className}`}>
      <div className="skeleton-table-header">
        {Array.from({ length: columns }).map((_, i) => (
          <div key={i} className="skeleton skeleton-table-cell skeleton-animate" />
        ))}
      </div>
      {Array.from({ length: rows }).map((_, rowIndex) => (
        <div key={rowIndex} className="skeleton-table-row">
          {Array.from({ length: columns }).map((_, colIndex) => (
            <div key={colIndex} className="skeleton skeleton-table-cell skeleton-animate" />
          ))}
        </div>
      ))}
    </div>
  );
}
