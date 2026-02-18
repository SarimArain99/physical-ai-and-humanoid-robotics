/**
 * Unified Navbar Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Consistent button component for all navbar elements.
 * Variants: primary, secondary, accent, icon
 */

import React from 'react';
import './styles.css';

/**
 * NavbarButton - Unified button component for navbar
 *
 * @param {Object} props
 * @param {string} props.variant - Button style variant: 'primary' | 'secondary' | 'accent' | 'icon'
 * @param {React.ReactNode} props.children - Button content (label or icon)
 * @param {string} props.className - Additional CSS classes
 * @param {boolean} props.loading - Show loading state
 * @param {boolean} props.disabled - Disable the button
 * @param {boolean} props.ripple - Add ripple effect on click
 * @param {Function} props.onClick - Click handler
 * @param {string} props.href - Link URL (renders as anchor if provided)
 * @param {boolean} props.hideLabelOnMobile - Hide text label on mobile, show icon only
 */
export default function NavbarButton({
  variant = 'primary',
  children,
  className = '',
  loading = false,
  disabled = false,
  ripple = true,
  onClick,
  href,
  hideLabelOnMobile = false,
  ...props
}) {
  const classes = [
    'navbar-btn',
    `navbar-btn--${variant}`,
    loading && 'navbar-btn--loading',
    ripple && 'navbar-btn--ripple',
    hideLabelOnMobile && 'navbar-btn--hide-label',
    className
  ].filter(Boolean).join(' ');

  const handleClick = (e) => {
    if (loading || disabled) {
      e.preventDefault();
      return;
    }
    onClick?.(e);
  };

  if (href) {
    return (
      <a
        href={href}
        className={classes}
        onClick={handleClick}
        disabled={disabled}
        {...props}
      >
        {children}
      </a>
    );
  }

  return (
    <button
      className={classes}
      onClick={handleClick}
      disabled={disabled || loading}
      {...props}
    >
      {children}
    </button>
  );
}

/**
 * Icon wrapper for navbar button icons
 */
export function NavbarButtonIcon({ children, className = '' }) {
  return (
    <span className={`navbar-btn__icon ${className}`}>
      {children}
    </span>
  );
}

/**
 * Label wrapper for navbar button text
 */
export function NavbarButtonLabel({ children, className = '' }) {
  return (
    <span className={`navbar-btn__label ${className}`}>
      {children}
    </span>
  );
}
