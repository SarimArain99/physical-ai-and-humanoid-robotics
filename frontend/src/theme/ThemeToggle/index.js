/**
 * Theme Toggle Component
 *
 * Custom theme switcher for dark/light/system modes.
 * Swizzles the default ColorModeToggle to have more control.
 */

import React from 'react';
import { useColorMode } from '@docusaurus/theme-common';

export default function ThemeToggle() {
  const { colorMode, setColorMode } = useColorMode();
  const [mounted, setMounted] = React.useState(false);

  // Avoid hydration mismatch
  React.useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return null;
  }

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
    if (colorMode === 'light') {
      return '🌙'; // Moon
    } else if (colorMode === 'dark') {
      return '☀️'; // Sun
    } else {
      return '💻'; // Computer/System
    }
  };

  const getTitle = () => {
    if (colorMode === 'light') return 'Switch to dark mode';
    if (colorMode === 'dark') return 'Switch to system preference';
    return 'Switch to light mode';
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
      <span style={{ marginRight: '0.3rem' }}>{getIcon()}</span>
    </button>
  );
}
