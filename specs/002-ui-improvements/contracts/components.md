# Component Contracts: UI/UX Improvements

**Feature**: 002-ui-improvements
**Date**: 2026-01-12
**Status**: Complete

## Overview

This document defines the interfaces, props, events, and behavioral contracts for all components in the UI/UX improvements feature.

---

## 1. IReadingProgress

### Purpose
Displays a horizontal progress bar fixed at the top of the content area showing the user's reading position within the current chapter.

### Props Interface

```typescript
interface IReadingProgressProps {
  /** Current scroll position as percentage (0-100) */
  progress: number;
  /** Optional color override (defaults to module primary color) */
  color?: string;
  /** Height of the progress bar in pixels (default: 4) */
  height?: number;
  /** Whether to show percentage text (default: false) */
  showLabel?: boolean;
  /** Callback when progress reaches 100% */
  onComplete?: () => void;
}
```

### Events

| Event | Trigger | Payload |
|-------|---------|---------|
| `onComplete` | Progress reaches 100% | `void` |

### Behavior Contract

1. **Initial State**: Progress bar is hidden (0% width) when page loads
2. **Updates**: Width updates on scroll events (throttled to 100ms)
3. **Completion**: `onComplete` callback fires once when progress first reaches 100%
4. **Styling**: Uses CSS transition for smooth width changes
5. **Accessibility**: Includes `role="progressbar"` and `aria-valuenow`

### Example Usage

```jsx
<ReadingProgress
  progress={scrollPercent}
  color="var(--module-primary)"
  height={4}
  showLabel={false}
  onComplete={() => markChapterComplete()}
/>
```

---

## 2. IChatIcon

### Purpose
Custom SVG robot icon for the chat widget toggle button, replacing the current emoji.

### Props Interface

```typescript
interface IChatIconProps {
  /** Whether the chat is currently open */
  isOpen: boolean;
  /** Number of unread messages (for badge) */
  unreadCount?: number;
  /** Whether to show pulsing animation */
  pulse?: boolean;
  /** Size in pixels (default: 24) */
  size?: number;
  /** Click handler */
  onClick?: () => void;
}
```

### Events

| Event | Trigger | Payload |
|-------|---------|---------|
| `onClick` | User clicks/taps icon | `MouseEvent \| TouchEvent` |

### Behavior Contract

1. **Icon Design**: Robot head shape with antenna, composed of SVG paths
2. **Animation**: Pulsing glow effect when `pulse={true}` (CSS keyframes)
3. **Badge**: Shows red circle with count when `unreadCount > 0`
4. **Hover**: Scale to 1.1 on hover (respects `prefers-reduced-motion`)
5. **Accessibility**: `aria-label` changes based on `isOpen` state

### SVG Specification

```xml
<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
  <!-- Robot head outline -->
  <rect x="6" y="4" width="12" height="14" rx="2" />
  <!-- Eyes -->
  <circle cx="9" cy="10" r="1.5" fill="currentColor" />
  <circle cx="15" cy="10" r="1.5" fill="currentColor" />
  <!-- Antenna -->
  <line x1="12" y1="4" x2="12" y2="1" />
  <circle cx="12" cy="1" r="1" />
  <!-- Mouth (smile when open) -->
  <path d={isOpen ? "M 8 14 Q 12 17 16 14" : "M 9 14 L 15 14"} />
</svg>
```

---

## 3. IHeroAnimation

### Purpose
Staggered animation reveal for hero section elements on homepage load.

### Props Interface

```typescript
interface IHeroAnimationProps {
  /** Array of element selectors to animate */
  elements: {
    selector: string;
    delay: number;      // Delay in ms
    animation: 'fadeIn' | 'slideUp' | 'scale';
  }[];
  /** Whether animations are enabled (from prefers-reduced-motion) */
  animationsEnabled?: boolean;
  /** Callback when all animations complete */
  onComplete?: () => void;
}
```

### Events

| Event | Trigger | Payload |
|-------|---------|---------|
| `onComplete` | All element animations finish | `void` |

### Behavior Contract

1. **Initialization**: Elements start with `opacity: 0` class
2. **Sequence**: Elements animate in order of `delay` property
3. **Animation Types**:
   - `fadeIn`: Fade from opacity 0 to 1
   - `slideUp`: Fade and translate Y from 20px to 0
   - `scale`: Scale from 0.9 to 1
4. **Duration**: Each animation lasts 300ms (respects reduced motion)
5. **Completion**: `onComplete` fires after last animation + 50ms buffer

### Example Usage

```jsx
<HeroAnimation
  elements={[
    { selector: '.hero__title', delay: 0, animation: 'slideUp' },
    { selector: '.hero__subtitle', delay: 100, animation: 'slideUp' },
    { selector: '.hero__cta', delay: 200, animation: 'fadeIn' },
  ]}
  animationsEnabled={!prefersReducedMotion}
  onComplete={() => console.log('Hero animated')}
/>
```

---

## 4. IChapterNavigation

### Purpose
Previous/Next chapter navigation links displayed at the bottom of content pages.

### Props Interface

```typescript
interface IChapterNavigationProps {
  /** Current chapter information */
  current: {
    id: string;
    title: string;
    slug: string;
  };
  /** Previous chapter (if exists) */
  prev?: {
    title: string;
    slug: string;
  };
  /** Next chapter (if exists) */
  next?: {
    title: string;
    slug: string;
  };
  /** Whether to show brief preview/description */
  showPreview?: boolean;
}
```

### Behavior Contract

1. **Layout**: Two buttons side-by-side on desktop, stacked on mobile
2. **Previous Button**: Left-aligned, points to previous chapter
3. **Next Button**: Right-aligned, points to next chapter
4. **Previews**: If `showPreview={true}`, displays chapter title below button text
5. **Disabled State**: Button visually disabled (but still linked) if no prev/next
6. **Icons**: Left arrow for prev, right arrow for next

### Example Usage

```jsx
<ChapterNavigation
  current={{ id: 'week1-intro', title: 'Introduction', slug: '/docs/week1/intro' }}
  prev={{ title: 'Course Overview', slug: '/docs/intro' }}
  next={{ title: 'Embodied Intelligence', slug: '/docs/week2/embodied-intelligence' }}
  showPreview={true}
/>
```

---

## 5. IModuleTheme

### Purpose
Dynamic theme provider that applies module-specific colors based on URL path.

### Props Interface

```typescript
interface IModuleThemeProps {
  children: React.ReactNode;
  /** Manual override for testing */
  overrideModule?: 'ros2' | 'digitalTwin' | 'isaac' | 'vla' | 'default';
}
```

### Behavior Contract

1. **Detection**: Parses `window.location.pathname` to determine module
2. **Application**: Sets `data-module` attribute on `<body>` element
3. **CSS Variables**: Updates CSS custom properties for module colors
4. **Transition**: Smooth color transition (300ms) when module changes
5. **Fallback**: Uses default theme if no pattern matches

### Module Detection Rules

| Path Pattern | Module |
|--------------|--------|
| `/docs/(intro\|week[1-5])/` | ros2 |
| `/docs/week[6-7]/` | digitalTwin |
| `/docs/week[8-10]/` | isaac |
| `/docs/week[11-13]/` | vla |
| (other) | default |

---

## 6. IThreeDRobot

### Purpose
Interactive 3D humanoid robot model on homepage with rotation controls.

### Props Interface

```typescript
interface IThreeDRobotProps {
  /** Path to GLTF/GLB model file */
  modelPath: string;
  /** Fallback image when WebGL unavailable */
  fallbackSrc: string;
  /** Auto-rotate animation speed (rad/sec) */
  autoRotateSpeed?: number;
  /** Enable zoom control */
  enableZoom?: boolean;
  /** Enable pan control */
  enablePan?: boolean;
  /** Maximum camera distance */
  maxDistance?: number;
  /** Minimum camera distance */
  minDistance?: number;
  /** Performance tier (auto-detected if not specified) */
  performanceTier?: 'low' | 'medium' | 'high';
}
```

### Events

| Event | Trigger | Payload |
|-------|---------|---------|
| `onLoad` | Model fully loaded | `void` |
| `onError` | Loading failed | `Error` |

### Behavior Contract

1. **Initialization**: Detect WebGL support on mount
2. **Fallback**: Render static SVG if WebGL unavailable
3. **Auto-Rotate**: Model rotates slowly when idle
4. **Interaction**: User can drag to rotate, scroll to zoom
5. **Performance**: Reduces quality on low-end devices
6. **Cleanup**: Disposes Three.js resources on unmount

---

## 7. IQuiz

### Purpose
Inline quiz component for knowledge checks after content sections.

### Props Interface

```typescript
interface IQuizProps {
  /** Unique quiz identifier */
  quizId: string;
  /** Quiz questions and answers */
  questions: Question[];
  /** Passing score percentage (0-100) */
  passingScore?: number;
  /** Whether to show results immediately */
  immediateFeedback?: boolean;
  /** Whether to shuffle question order */
  shuffle?: boolean;
}

interface Question {
  id: string;
  type: 'multiple-choice' | 'true-false';
  question: string;
  options: string[];
  correctIndex: number;
  explanation: string;
}
```

### Events

| Event | Trigger | Payload |
|-------|---------|---------|
| `onComplete` | User submits all answers | `{ score: number, passed: boolean }` |
| `onRetry` | User clicks "Try Again" | `void` |

### Behavior Contract

1. **Initial State**: Show questions with radio buttons for options
2. **Selection**: User can change selection before submitting
3. **Submission**: Disable inputs, show feedback for each answer
4. **Scoring**: Calculate percentage of correct answers
5. **Results**: Show score, pass/fail message, "Try Again" button
6. **Persistence**: Save best score to localStorage

### LocalStorage Schema

```json
{
  "quiz-quiz-id": {
    "bestScore": 80,
    "lastAttempt": "2026-01-12T10:00:00Z",
    "attempts": 3
  }
}
```

---

## 8. IVisualEffects

### Purpose
Particle background and scroll-triggered animations with performance awareness.

### Props Interface

```typescript
interface IVisualEffectsProps {
  children: React.ReactNode;
  /** Enable particle background */
  particles?: boolean;
  /** Enable scroll animations */
  scrollAnimations?: boolean;
  /** Force performance tier (for testing) */
  forceTier?: 'low' | 'medium' | 'high';
}
```

### Behavior Contract

1. **Particles**:
   - Canvas-based rendering for performance
   - Particle count varies by performance tier (100/50/0)
   - Pauses when tab not visible (Page Visibility API)
2. **Scroll Animations**:
   - Uses Intersection Observer for viewport detection
   - Animations trigger once per element
   - Respects `prefers-reduced-motion`
3. **Performance Detection**:
   - Checks `navigator.hardwareConcurrency` and `deviceMemory`
   - Automatically reduces effects on low-end devices
4. **Cleanup**: Removes event listeners and animation frames on unmount

---

## 9. ISkeletonLoader

### Purpose
Loading skeleton for async operations (translation, personalization).

### Props Interface

```typescript
interface ISkeletonLoaderProps {
  /** Type of content being loaded */
  variant: 'text' | 'heading' | 'card' | 'custom';
  /** Number of skeleton lines (for text variant) */
  lines?: number;
  /** Width (for custom variant) */
  width?: string;
  /** Height (for custom variant) */
  height?: string;
  /** Animation type */
  animation?: 'pulse' | 'wave' | 'none';
}
```

### Behavior Contract

1. **Visual**: Gray placeholder with shimmer animation
2. **Animation**: Pulse or wave effect to indicate loading
3. **Accessibility**: `role="status"` with `aria-label="Loading..."`
4. **Performance**: CSS-only animation, no JS overhead

---

## 10. IPageTransition

### Purpose
Wrapper component for smooth page transition animations.

### Props Interface

```typescript
interface IPageTransitionProps {
  children: React.ReactNode;
  /** Transition type */
  type?: 'fade' | 'slide' | 'scale';
  /** Duration in milliseconds */
  duration?: number;
}
```

### Behavior Contract

1. **Mount**: Applies exit animation to previous page content
2. **Unmount**: Applies enter animation to new page content
3. **Duration**: Default 300ms, configurable
4. **Easing**: Uses `cubic-bezier(0.4, 0, 0.2, 1)` for smooth feel
5. **Reduced Motion**: Skips animation if `prefers-reduced-motion` is true

---

## Integration Points

### Docusaurus Swizzable Components

The following Docusaurus components will be swizzled to apply our enhancements:

| Component | Purpose | Enhancement |
|-----------|---------|-------------|
| `@theme/DocPage` | Main content page | Add ReadingProgress, ChapterNavigation |
| `@theme/Footer` | Page footer | No changes needed |
| `@theme/Navbar` | Navigation bar | Add module indicator |
| `@theme/DocSidebar` | Sidebar navigation | Add module icons |

### Client Module Structure

```javascript
// src/clientModules.js
export default {
  // Module theme provider wraps entire app
  ModuleThemeProvider: {
    component: '@site/src/components/ModuleThemeProvider',
  },
};
```

---

## CSS Custom Properties Contract

Components MUST use the following CSS custom properties for consistency:

```css
/* Colors */
--module-primary: /* Current module primary color */
--module-accent: /* Current module accent color */
--color-bg: /* Background color */
--color-surface: /* Surface/card color */
--color-text: /* Text color */

/* Spacing */
--space-xs: 4px
--space-sm: 8px
--space-md: 16px
--space-lg: 24px
--space-xl: 32px

/* Animation */
--duration-fast: 150ms
--duration-normal: 300ms
--duration-slow: 500ms
--ease-out: cubic-bezier(0.215, 0.61, 0.355, 1)
```
