/**
 * Theme Toggle Component
 * Feature: 002-ui-improvements
 * Task: Working theme toggle (dark/light/system)
 *
 * A custom theme toggle that switches between light, dark, and auto (system) modes.
 */

import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useColorMode } from '@docusaurus/theme-common';
import './styles.css';

// Icons
const SunIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <circle cx="12" cy="12" r="5"></circle>
    <line x1="12" y1="1" x2="12" y2="3"></line>
    <line x1="12" y1="21" x2="12" y2="23"></line>
    <line x1="4.22" y1="4.22" x2="5.64" y2="5.64"></line>
    <line x1="18.36" y1="18.36" x2="19.78" y2="19.78"></line>
    <line x1="1" y1="12" x2="3" y2="12"></line>
    <line x1="21" y1="12" x2="23" y2="12"></line>
    <line x1="4.22" y1="19.78" x2="5.64" y2="18.36"></line>
    <line x1="18.36" y1="5.64" x2="19.78" y2="4.22"></line>
  </svg>
);

const MoonIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path>
  </svg>
);

const SystemIcon = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect>
    <line x1="8" y1="21" x2="16" y2="21"></line>
    <line x1="12" y1="17" x2="12" y2="21"></line>
  </svg>
);

const CheckIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="3" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="20 6 9 17 4 12"></polyline>
  </svg>
);

/**
 * Theme toggle modes: 'light' | 'dark' | 'system'
 */
const MODES = ['light', 'dark', 'system'];

function ThemeToggleContent() {
  const { colorMode, setColorMode } = useColorMode();
  const [isOpen, setIsOpen] = useState(false);
  const [currentMode, setCurrentMode] = useState('system');

  // Initialize mode from localStorage or colorMode
  useEffect(() => {
    const stored = localStorage.getItem('theme');
    if (stored && MODES.includes(stored)) {
      setCurrentMode(stored);
    } else {
      setCurrentMode('system');
    }
  }, []);

  // Update mode when colorMode changes externally
  useEffect(() => {
    const stored = localStorage.getItem('theme');
    if (stored === 'light' && colorMode === 'light') {
      setCurrentMode('light');
    } else if (stored === 'dark' && colorMode === 'dark') {
      setCurrentMode('dark');
    } else if (!stored || stored === 'system') {
      setCurrentMode('system');
    }
  }, [colorMode]);

  const handleModeSelect = (mode) => {
    setCurrentMode(mode);
    localStorage.setItem('theme', mode);

    if (mode === 'system') {
      // Clear stored preference and let Docusaurus use system preference
      localStorage.removeItem('docusaurus.theme');
      // Reload to apply system preference
      window.location.reload();
    } else {
      setColorMode(mode);
    }
    setIsOpen(false);
  };

  const getCurrentIcon = () => {
    if (currentMode === 'light') return <SunIcon />;
    if (currentMode === 'dark') return <MoonIcon />;
    return <SystemIcon />;
  };

  const getLabel = () => {
    if (currentMode === 'light') return 'Light';
    if (currentMode === 'dark') return 'Dark';
    return 'Auto';
  };

  return (
    <div className="theme-toggle-dropdown">
      <button
        className="navbar-btn navbar-btn--icon theme-toggle-btn"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle theme"
        aria-expanded={isOpen}
        title={`Current theme: ${getLabel()}`}
      >
        <span className="theme-toggle-icon">{getCurrentIcon()}</span>
      </button>

      {isOpen && (
        <div className="theme-toggle-menu" role="menu">
          <button
            className={`theme-toggle-item ${currentMode === 'light' ? 'active' : ''}`}
            onClick={() => handleModeSelect('light')}
            role="menuitem"
          >
            <span className="theme-toggle-item-icon"><SunIcon /></span>
            <span className="theme-toggle-item-label">Light</span>
            {currentMode === 'light' && <span className="theme-toggle-check"><CheckIcon /></span>}
          </button>

          <button
            className={`theme-toggle-item ${currentMode === 'dark' ? 'active' : ''}`}
            onClick={() => handleModeSelect('dark')}
            role="menuitem"
          >
            <span className="theme-toggle-item-icon"><MoonIcon /></span>
            <span className="theme-toggle-item-label">Dark</span>
            {currentMode === 'dark' && <span className="theme-toggle-check"><CheckIcon /></span>}
          </button>

          <button
            className={`theme-toggle-item ${currentMode === 'system' ? 'active' : ''}`}
            onClick={() => handleModeSelect('system')}
            role="menuitem"
          >
            <span className="theme-toggle-item-icon"><SystemIcon /></span>
            <span className="theme-toggle-item-label">System</span>
            {currentMode === 'system' && <span className="theme-toggle-check"><CheckIcon /></span>}
          </button>
        </div>
      )}
    </div>
  );
}

export default function ThemeToggle() {
  return (
    <BrowserOnly fallback={null}>
      {() => <ThemeToggleContent />}
    </BrowserOnly>
  );
}
