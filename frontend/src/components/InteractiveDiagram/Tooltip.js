import React from 'react';
import styles from './styles.css';

/**
 * Diagram Tooltip Component
 * Feature: 002-ui-improvements
 * Task: T066
 *
 * Tooltip for displaying node information on hover
 */

export default function DiagramTooltip({ node, position, onClose }) {
  if (!node) return null;

  const style = position ? {
    left: `${position.x}px`,
    top: `${position.y}px`
  } : {};

  return (
    <div
      className={styles.tooltip}
      style={style}
      role="tooltip"
      aria-live="polite"
    >
      <strong>{node.label}</strong>
      <p>{node.description}</p>
      {node.details && node.details.length > 0 && (
        <ul style={{ margin: '0.5rem 0 0 0', padding: '0 0 0 1rem', fontSize: '0.75rem' }}>
          {node.details.slice(0, 2).map((detail, i) => (
            <li key={i} style={{ color: 'var(--color-text-tertiary, #64748B)' }}>
              {detail}
            </li>
          ))}
        </ul>
      )}
      {onClose && (
        <button
          onClick={onClose}
          className={styles.closeButton}
          aria-label="Close tooltip"
          style={{
            position: 'absolute',
            top: '0.25rem',
            right: '0.25rem',
            width: '20px',
            height: '20px',
            padding: '0',
            background: 'transparent',
            border: 'none',
            color: 'inherit',
            cursor: 'pointer',
            fontSize: '0.875rem'
          }}
        >
          ×
        </button>
      )}
    </div>
  );
}
