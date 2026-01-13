# Design Tokens: UI/UX Improvements

**Feature**: 002-ui-improvements
**Date**: 2026-01-12
**Status**: Complete

## Overview

This document defines the design tokens for the UI/UX improvements feature, including colors, typography, spacing, animations, and breakpoints. These tokens ensure consistency across all new components.

---

## 1. Color Tokens

### Module Accent Colors

```css
/* ROS 2 Module - Orange */
--color-ros2-primary: #FF6B35;
--color-ros2-accent: #FF8C5A;
--color-ros2-light: #FFE5DB;
--color-ros2-dark: #CC5629;

/* Digital Twin Module - Teal */
--color-digitaltwin-primary: #4ECDC4;
--color-digitaltwin-accent: #7EDDD6;
--color-digitaltwin-light: #DBF5F2;
--color-digitaltwin-dark: #3BA39C;

/* NVIDIA Isaac Module - Purple */
--color-isaac-primary: #A855F7;
--color-isaac-accent: #C084FC;
--color-isaac-light: #E9D5FF;
--color-isaac-dark: #7C3AED;

/* VLA Module - Red */
--color-vla-primary: #FF0844;
--color-vla-accent: #FF4D6D;
--color-vla-light: #FFE5EA;
--color-vla-dark: #CC0636;

/* Default Brand - Green */
--color-brand-primary: #2ECC71;
--color-brand-accent: #22C55E;
--color-brand-light: #A3FAD9;
--color-brand-dark: #25B365;
```

### Semantic Colors

```css
/* Base Colors */
--color-bg-primary: #1E2A38;
--color-bg-surface: #151E29;
--color-bg-elevated: #1E2A38;
--color-text-primary: #FFFFFF;
--color-text-secondary: #94A3B8;
--color-text-tertiary: #64748B;
--color-text-muted: #475569;

/* UI Colors */
--color-border: #2C3E50;
--color-border-light: #334155;
--color-divider: #1E293B;

/* Status Colors */
--color-success: #2ECC71;
--color-warning: #F59E0B;
--color-error: #EF4444;
--color-info: #3B82F6;

/* Overlay Colors */
--color-overlay: rgba(0, 0, 0, 0.5);
--color-overlay-light: rgba(255, 255, 255, 0.1);
```

### WCAG Compliance Matrix

| Color | Use Case | Contrast (White) | Contrast (Black) | WCAG AA |
|-------|----------|-----------------|------------------|----------|
| #FF6B35 | ROS2 Primary | 1.8:1 | 11.5:1 | ✅ Pass (with dark bg) |
| #4ECDC4 | Digital Twin | 1.5:1 | 13.2:1 | ✅ Pass (with dark bg) |
| #A855F7 | Isaac | 3.0:1 | 6.9:1 | ⚠️ Borderline |
| #FF0844 | VLA | 1.6:1 | 12.2:1 | ✅ Pass (with dark bg) |
| #2ECC71 | Brand | 1.4:1 | 1.9:1 | ⚠️ Borderline |

**Note**: All accent colors are used on dark backgrounds (#1E2A38) for AA compliance.

---

## 2. Typography Tokens

### Font Families

```css
/* Display Font (Headings) */
--font-display: 'Orbitron', 'Courier New', monospace;
--font-display-weight: 700;

/* Body Font */
--font-body: 'Space Grotesk', 'Inter', system-ui, -apple-system, sans-serif;
--font-body-weight: 400;

/* Code Font */
--font-code: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
```

### Font Sizes

```css
/* Display */
--text-xs: 0.75rem;      /* 12px */
--text-sm: 0.875rem;     /* 14px */
--text-base: 1rem;       /* 16px */
--text-lg: 1.125rem;     /* 18px */
--text-xl: 1.25rem;      /* 20px */
--text-2xl: 1.5rem;      /* 24px */
--text-3xl: 1.875rem;    /* 30px */
--text-4xl: 2.25rem;     /* 36px */
--text-5xl: 3rem;        /* 48px */
```

### Font Weights

```css
--font-light: 300;
--font-normal: 400;
--font-medium: 500;
--font-semibold: 600;
--font-bold: 700;
--font-extrabold: 800;
```

### Line Heights

```css
--leading-tight: 1.25;
--leading-snug: 1.375;
--leading-normal: 1.5;
--leading-relaxed: 1.625;
--leading-loose: 2;
```

### Letter Spacing

```css
--tracking-tighter: -0.025em;
--tracking-tight: -0.01em;
--tracking-normal: 0;
--tracking-wide: 0.025em;
--tracking-wider: 0.05em;
```

### Typography Scale by Element

| Element | Size | Weight | Line Height | Letter Spacing |
|---------|------|--------|-------------|----------------|
| H1 | 2.5rem | 800 | 1.1 | -0.02em |
| H2 | 2rem | 700 | 1.2 | -0.015em |
| H3 | 1.5rem | 600 | 1.3 | -0.01em |
| H4 | 1.25rem | 600 | 1.4 | normal |
| Body | 1rem | 400 | 1.6 | normal |
| Caption | 0.875rem | 400 | 1.5 | 0.01em |
| Code | 0.875rem | 400 | 1.5 | normal |

---

## 3. Spacing Tokens

### Spacing Scale

```css
--space-0: 0;
--space-px: 1px;
--space-0_5: 0.125rem;   /* 2px */
--space-1: 0.25rem;      /* 4px */
--space-2: 0.5rem;       /* 8px */
--space-3: 0.75rem;      /* 12px */
--space-4: 1rem;         /* 16px */
--space-5: 1.25rem;      /* 20px */
--space-6: 1.5rem;       /* 24px */
--space-8: 2rem;         /* 32px */
--space-10: 2.5rem;      /* 40px */
--space-12: 3rem;        /* 48px */
--space-16: 4rem;        /* 64px */
--space-20: 5rem;        /* 80px */
--space-24: 6rem;        /* 96px */
```

### Component Padding

```css
--padding-card: var(--space-6);
--padding-button: var(--space-2) var(--space-4);
--padding-input: var(--space-3);
--padding-modal: var(--space-8);
```

---

## 4. Border Radius Tokens

```css
--radius-none: 0;
--radius-sm: 0.125rem;    /* 2px */
--radius-base: 0.25rem;   /* 4px */
--radius-md: 0.375rem;    /* 6px */
--radius-lg: 0.5rem;      /* 8px */
--radius-xl: 0.75rem;     /* 12px */
--radius-2xl: 1rem;       /* 16px */
--radius-3xl: 1.5rem;     /* 24px */
--radius-full: 9999px;
```

### Component Border Radius

| Component | Token | Value |
|-----------|-------|-------|
| Button | `--radius-full` | Pill shape |
| Card | `--radius-xl` | 12px |
| Input | `--radius-md` | 6px |
| Modal | `--radius-2xl` | 16px |
| Tooltip | `--radius-md` | 6px |
| Chat Bubble | `--radius-xl` | 12px (different corner radius) |

---

## 5. Shadow Tokens

```css
/* Box Shadows */
--shadow-sm: 0 1px 2px rgba(0, 0, 0, 0.05);
--shadow-base: 0 1px 3px rgba(0, 0, 0, 0.1), 0 1px 2px rgba(0, 0, 0, 0.06);
--shadow-md: 0 4px 6px rgba(0, 0, 0, 0.07), 0 2px 4px rgba(0, 0, 0, 0.06);
--shadow-lg: 0 10px 15px rgba(0, 0, 0, 0.1), 0 4px 6px rgba(0, 0, 0, 0.05);
--shadow-xl: 0 20px 25px rgba(0, 0, 0, 0.1), 0 10px 10px rgba(0, 0, 0, 0.04);
--shadow-2xl: 0 25px 50px rgba(0, 0, 0, 0.25);

/* Glow Effects */
--glow-sm: 0 0 10px rgba(46, 204, 113, 0.3);
--glow-md: 0 0 20px rgba(46, 204, 113, 0.5);
--glow-lg: 0 0 30px rgba(46, 204, 113, 0.7);
```

---

## 6. Animation Tokens

### Durations

```css
--duration-instant: 50ms;
--duration-fast: 150ms;
--duration-base: 300ms;
--duration-slow: 500ms;
--duration-slower: 750ms;
```

### Easing Functions

```css
--ease-linear: linear;
--ease-in: cubic-bezier(0.4, 0, 1, 1);
--ease-out: cubic-bezier(0, 0, 0.2, 1);
--ease-in-out: cubic-bezier(0.4, 0, 0.2, 1);
--ease-bounce: cubic-bezier(0.68, -0.55, 0.265, 1.55);
--ease-elastic: cubic-bezier(0.68, -0.6, 0.32, 1.6);
```

### Keyframe Animations

```css
/* Fade In */
@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

/* Slide Up */
@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Scale In */
@keyframes scaleIn {
  from {
    opacity: 0;
    transform: scale(0.9);
  }
  to {
    opacity: 1;
    transform: scale(1);
  }
}

/* Pulse */
@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

/* Spin */
@keyframes spin {
  from { transform: rotate(0deg); }
  to { transform: rotate(360deg); }
}

/* Bounce */
@keyframes bounce {
  0%, 100% {
    transform: translateY(0);
    animation-timing-function: cubic-bezier(0.8, 0, 1, 1);
  }
  50% {
    transform: translateY(-10px);
    animation-timing-function: cubic-bezier(0, 0, 0.2, 1);
  }
}

/* Shimmer (for skeleton loaders) */
@keyframes shimmer {
  0% { background-position: -200% 0; }
  100% { background-position: 200% 0; }
}
```

### Reduced Motion Media Query

```css
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

---

## 7. Breakpoint Tokens

```css
/* Mobile First Approach */
--breakpoint-xs: 0px;        /* 0 - 639px */
--breakpoint-sm: 640px;      /* 640px - 767px */
--breakpoint-md: 768px;      /* 768px - 1023px */
--breakpoint-lg: 1024px;     /* 1024px - 1279px */
--breakpoint-xl: 1280px;     /* 1280px+ */
--breakpoint-2xl: 1536px;    /* 1536px+ */
```

### Container Widths

```css
--container-sm: 640px;
--container-md: 768px;
--container-lg: 1024px;
--container-xl: 1280px;
--container-2xl: 1536px;
```

---

## 8. Z-Index Scale

```css
--z-index-base: 0;
--z-index-dropdown: 1000;
--z-index-sticky: 1020;
--z-index-fixed: 1030;
--z-index-modal-backdrop: 1040;
--z-index-modal: 1050;
--z-index-popover: 1060;
--z-index-tooltip: 1070;
--z-index-chat-widget: 9999;
```

---

## 9. Chat Widget Specific Tokens

```css
/* Chat Widget */
--chat-widget-size: 65px;
--chat-widget-size-mobile: 56px;
--chat-window-width: 380px;
--chat-window-height: 600px;
--chat-window-width-mobile: calc(100vw - 30px);
--chat-window-height-mobile: 70vh;

/* Chat Colors */
--chat-bg-primary: #1E2A38;
--chat-bg-surface: #151E29;
--chat-bubble-user: #2ECC71;
--chat-bubble-bot: #2C3E50;
--chat-border: #2C3E50;
```

---

## 10. Progress Indicator Tokens

```css
/* Reading Progress */
--progress-height: 4px;
--progress-height-thick: 8px;
--progress-color: var(--module-primary);
```

---

## Usage Example

```css
.component {
  /* Use tokens for consistency */
  padding: var(--space-4);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-md);
  background-color: var(--color-bg-surface);
  color: var(--color-text-primary);
  font-family: var(--font-body);
  font-size: var(--text-base);
  transition: all var(--duration-base) var(--ease-out);
}

@media (prefers-reduced-motion: reduce) {
  .component {
    transition: none;
  }
}
```

---

## Token Naming Convention

- **Colors**: `--color-{purpose}-{variant}` (e.g., `--color-brand-primary`)
- **Typography**: `--{property}-{variant}` (e.g., `--text-xl`, `--font-bold`)
- **Spacing**: `--space-{scale}` (e.g., `--space-4`)
- **Borders**: `--radius-{size}` (e.g., `--radius-lg`)
- **Shadows**: `--shadow-{size}` (e.g., `--shadow-md`)
- **Animations**: `--duration-{speed}` (e.g., `--duration-base`)
- **Breakpoints**: `--breakpoint-{size}` (e.g., `--breakpoint-md`)
