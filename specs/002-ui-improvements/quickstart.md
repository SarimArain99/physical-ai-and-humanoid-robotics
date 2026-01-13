# Quickstart Guide: UI/UX Improvements

**Feature**: 002-ui-improvements
**Date**: 2026-01-13
**Status**: Complete (T091: Updated with implementation findings)

## Overview

This guide helps developers quickly implement and test the UI/UX improvements for the AI-Native Textbook.

---

## Prerequisites

- Node.js 18+ installed
- Docusaurus site already set up at `frontend/`
- Git branch `002-ui-improvements` checked out

---

## Quick Setup (5 minutes)

### 1. Install Dependencies

```bash
cd frontend
npm install three @types/three react-intersection-observer
```

### 2. Add Google Fonts

Edit `frontend/docusaurus.config.ts`:

```typescript
const config = {
  themeConfig: {
    customCss: [
      require.resolve('./src/css/fonts.css'),
      require.resolve('./src/css/theme.css'),
      require.resolve('./src/css/animations.css'),
    ],
  },
  scripts: [
    {
      src: 'https://fonts.googleapis.com',
      async: true,
    },
  ],
};
```

### 3. Create Font CSS

Create `frontend/src/css/fonts.css`:

```css
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;500;600;700;800;900&family=Space+Grotesk:wght@300;400;500;600;700&display=swap');

:root {
  --font-display: 'Orbitron', sans-serif;
  --font-body: 'Space Grotesk', system-ui, -apple-system, sans-serif;
}

h1, h2, h3, h4, h5, h6 {
  font-family: var(--font-display);
}

body, p, span, div {
  font-family: var(--font-body);
}
```

### 4. Test the Build

```bash
npm run build
npm run serve
```

Visit `http://localhost:3000` to see the new fonts applied.

---

## Component Implementation

### P1: Chat Icon (Custom SVG)

Replace emoji in `frontend/src/components/ChatWidget/index.js`:

```javascript
import React from 'react';

const RobotIcon = ({ size = 24, className = '' }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    className={className}
  >
    {/* Robot head */}
    <rect x="6" y="4" width="12" height="14" rx="2" />
    {/* Eyes */}
    <circle cx="9" cy="10" r="1.5" fill="currentColor" />
    <circle cx="15" cy="10" r="1.5" fill="currentColor" />
    {/* Antenna */}
    <line x1="12" y1="4" x2="12" y2="1" />
    <circle cx="12" cy="1" r="1" fill="currentColor" />
    {/* Mouth */}
    <path d="M 9 14 Q 12 17 15 14" />
  </svg>
);

// Use in component:
<RobotIcon size={24} className="chat-icon" />
```

### P1: Reading Progress

Create `frontend/src/components/ReadingProgress/index.js`:

```javascript
import React, { useState, useEffect } from 'react';
import './styles.css';

const ReadingProgress = () => {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const windowHeight = window.innerHeight;
      const documentHeight = document.documentElement.scrollHeight;
      const scrolled = window.scrollY;
      const maxScroll = documentHeight - windowHeight;
      const percent = Math.min((scrolled / maxScroll) * 100, 100);
      setProgress(percent);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <div className="reading-progress-bar">
      <div className="reading-progress-fill" style={{ width: `${progress}%` }} />
    </div>
  );
};

export default ReadingProgress;
```

Create `frontend/src/components/ReadingProgress/styles.css`:

```css
.reading-progress-bar {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 4px;
  background-color: var(--ifm-background-surface-color);
  z-index: 9999;
}

.reading-progress-fill {
  height: 100%;
  background-color: var(--module-primary, #2ECC71);
  transition: width 0.1s linear;
}
```

### P2: Module Theme Provider

Create `frontend/src/components/ModuleThemeProvider/index.js`:

```javascript
import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const MODULE_COLORS = {
  ros2: '#FF6B35',
  digitalTwin: '#4ECDC4',
  isaac: '#A855F7',
  vla: '#FF0844',
};

const detectModule = (path) => {
  if (/^\/docs\/(intro|week[1-5])/.test(path)) return 'ros2';
  if (/^\/docs\/week[6-7]/.test(path)) return 'digitalTwin';
  if (/^\/docs\/week[8-10]/.test(path)) return 'isaac';
  if (/^\/docs\/week[11-13]/.test(path)) return 'vla';
  return 'default';
};

export default function ModuleThemeProvider({ children }) {
  const location = useLocation();
  const module = detectModule(location.pathname);

  useEffect(() => {
    document.body.setAttribute('data-module', module);
    document.documentElement.style.setProperty(
      '--module-primary',
      MODULE_COLORS[module] || '#2ECC71'
    );
  }, [module]);

  return <>{children}</>;
}
```

Wrap in `frontend/src/theme/Layout/index.js`:

```javascript
import ModuleThemeProvider from '@site/src/components/ModuleThemeProvider';

export default function Layout(props) {
  return (
    <ModuleThemeProvider>
      <OriginalLayout {...props} />
    </ModuleThemeProvider>
  );
}
```

---

## Testing Checklist

### Visual Testing

- [ ] Fonts load correctly on homepage
- [ ] Reading progress bar fills when scrolling
- [ ] Chat icon shows custom SVG (not emoji)
- [ ] Module colors change when navigating between weeks
- [ ] Week cards have hover effects

### Accessibility Testing

- [ ] All animations disabled with `prefers-reduced-motion`
- [ ] Color contrast meets WCAG AA (use axe DevTools)
- [ ] Keyboard navigation works for all interactive elements
- [ ] Screen reader announces progress changes

### Performance Testing

- [ ] Homepage loads within 1 second (Lighthouse)
- [ ] No layout shifts (CLS < 0.1)
- [ ] Fonts use `font-display: swap` for FOUT prevention

---

## Troubleshooting

### Fonts Not Loading

**Problem**: Default fonts showing instead of Orbitron/Space Grotesk

**Solution**: Check browser console for CORS errors. Ensure `docusaurus.config.ts` has `preconnect` scripts:

```typescript
scripts: [
  { src: 'https://fonts.googleapis.com', async: true },
  { src: 'https://fonts.gstatic.com', async: true },
],
```

### Module Theme Not Applying

**Problem**: Colors not changing when navigating pages

**Solution**: Verify `ModuleThemeProvider` wraps the Layout component. Check `data-module` attribute on body element:

```javascript
console.log(document.body.getAttribute('data-module'));
```

### Reading Progress Bar Not Visible

**Problem**: Progress bar not appearing at top of page

**Problem**: Progress bar not appearing at top of page

**Solution**: Check z-index conflict. Ensure z-index is 9999 or higher:

```css
.reading-progress-bar {
  z-index: 9999 !important;
}
```

### 3D Model Not Rendering (P4)

**Problem**: Blank space where 3D model should be

**Solution**: Check browser console for WebGL errors. Verify model path is correct. Fallback should show if WebGL unavailable:

```javascript
const canvas = document.createElement('canvas');
if (!canvas.getContext('webgl2') && !canvas.getContext('webgl')) {
  console.log('WebGL not supported, showing fallback');
}
```

---

## Development Workflow

1. **Start Dev Server**:
   ```bash
   cd frontend && npm run start
   ```

2. **Watch for Changes**:
   - CSS changes auto-reload
   - Component changes hot-reload
   - Docusaurus config changes require restart

3. **Build for Production**:
   ```bash
   npm run build
   ```

4. **Deploy**:
   ```bash
   npm run deploy
   ```

---

## Priority Implementation Order

### First (P1 - HIGH)
1. Add Google Fonts
2. Create custom SVG chat icon
3. Implement reading progress
4. Add hero animations

### Second (P2 - MEDIUM)
1. Module theme provider
2. Sidebar icons
3. Chapter navigation
4. Page transitions
5. Skeleton loaders

### Third (P3 - POLISH)
1. Advanced hover effects
2. Micro-interactions

### Fourth (P4 - ADVANCED)
1. Three.js 3D robot
2. Interactive diagrams
3. Quiz components
4. Particle backgrounds

---

## File Structure Reference

```
frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget/           # Enhanced with SVG icon
│   │   ├── ReadingProgress/      # NEW: Progress bar
│   │   ├── HeroAnimation/        # NEW: Staggered animations
│   │   ├── ModuleThemeProvider/ # NEW: Dynamic theming
│   │   ├── ChapterNavigation/    # NEW: Prev/Next links
│   │   ├── ThreeDRobot/          # NEW: 3D model (P4)
│   │   └── Quiz/                  # NEW: Quizzes (P4)
│   ├── css/
│   │   ├── fonts.css             # NEW: Font imports
│   │   ├── theme.css              # NEW: Module theming
│   │   └── animations.css        # NEW: Keyframes
│   └── utils/
│       ├── moduleDetection.js    # NEW: Path parser
│       └── performance.js         # NEW: Device detection
└── docusaurus.config.ts          # UPDATE: Add fonts
```

---

## Next Steps

1. Implement P1 components for immediate visual impact
2. Test across browsers (Chrome, Firefox, Safari, Edge)
3. Run accessibility audits (axe DevTools)
4. Create visual regression tests (Playwright)
5. Deploy to staging environment for user feedback

---

## Implementation Notes (T091)

### CSS Module Import Fix

**Issue**: Webpack warnings about `export 'default' (imported as 'styles') was not found`

**Solution**: Use side-effect CSS imports instead of default imports:

```javascript
// ❌ Wrong (causes warnings)
import styles from './styles.css';
<div className={styles.container}>

// ✅ Correct (side-effect import)
import './styles.css';
<div className="container">
```

Or use CSS Modules with proper naming (`.module.css` suffix).

### Lazy Loading for Heavy Components

**Issue**: Three.js component (ThreeDRobot) adds significant bundle size

**Solution**: Use React.lazy for code splitting:

```javascript
import { lazy, Suspense } from 'react';

const ThreeDRobot = lazy(() => import('@site/src/components/ThreeDRobot'));

// Wrap with Suspense fallback
<Suspense fallback={<div>Loading...</div>}>
  <ThreeDRobot />
</Suspense>
```

### Module Color Transitions

**Discovery**: Theme transitions need explicit transition CSS for smooth module color changes

**Solution**: Add transition class when module changes:

```javascript
// ModuleThemeProvider adds/removes this class
body.theme-transitioning {
  transition: background-color 300ms var(--ease-out),
              color 300ms var(--ease-out),
              border-color 300ms var(--ease-out);
}
```

### Progress Completion Tracking

**Discovery**: ReadingProgress needs to save to localStorage on 100% scroll

**Solution**: Dispatch custom event for other components:

```javascript
if (percent >= 100 && !hasSavedCompletion) {
  markChapterProgress(location.pathname, 100);
  hasSavedCompletion = true;
  window.dispatchEvent(new CustomEvent('chapter-completed', {
    detail: { path: location.pathname, timestamp: Date.now() }
  }));
}
```

### WCAG AA Contrast Considerations

**Discovery**: Module colors have insufficient contrast with white text for normal text

**Solution**: Use module colors for accents only, not primary text:

| Color | Contrast (on white) | Use Case |
|-------|-------------------|----------|
| #2ECC71 (Green) | 1.47:1 | Accents, buttons with dark text |
| #FF6B35 (Orange) | 1.96:1 | Accents, borders, glows |
| #4ECDC4 (Teal) | 1.39:1 | Accents, highlights |
| #A855F7 (Purple) | 2.82:1 | Large text only |
| #FF0844 (Red) | 2.04:1 | Large text only |
