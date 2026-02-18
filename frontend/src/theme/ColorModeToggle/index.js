/**
 * Color Mode Toggle (Theme Switcher)
 *
 * Swizzled default ColorModeToggle to customize behavior.
 * This component cycles through light -> dark -> system modes.
 */

import React from 'react';
import { useColorMode, useThemeConfig } from '@docusaurus/theme-common';

export default function ColorModeToggle() {
  const { colorMode, setColorMode } = useColorMode();
  const { minification } = useThemeConfig();

  const cycleColorMode = () => {
    if (colorMode === 'light') {
      setColorMode('dark');
    } else if (colorMode === 'dark') {
      setColorMode('system');
    } else {
      setColorMode('light');
    }
  };

  const getIcon = () => {
    switch (colorMode) {
      case 'light':
        return '\u{1F311}'; // 🌑 New moon (click to go dark)
      case 'dark':
        return '\u{1F304}'; // 🌄 Sunrise (click to use system)
      default: // system
        return '\u{1F4BB}'; // 💻 Computer (click to go light)
    }
  };

  const getTitle = () => {
    switch (colorMode) {
      case 'light':
        return 'Switch to dark mode';
      case 'dark':
        return 'Switch to system preference';
      default:
        return 'Switch to light mode';
    }
  };

  return (
    <button
      onClick={cycleColorMode}
      title={getTitle()}
      aria-label="Toggle color mode"
      style={{
        backgroundColor: 'transparent',
        border: 'none',
        cursor: 'pointer',
        fontSize: '1.2rem',
        padding: '0.5rem',
        display: 'flex',
        alignItems: 'center',
        borderRadius: '4px',
        transition: 'background-color 0.2s',
      }}
      onMouseEnter={(e) => e.currentTarget.style.backgroundColor = 'rgba(0,0,0,0.05)'}
      onMouseLeave={(e) => e.currentTarget.style.backgroundColor = 'transparent'}
    >
      {getIcon()}
    </button>
  );
}
