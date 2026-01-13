# Data Model: UI/UX Improvements

**Feature**: 002-ui-improvements
**Date**: 2026-01-12
**Status**: Complete

## Overview

This document defines the data structures and state entities for the UI/UX improvements feature. Since this is a frontend-only enhancement, all state is managed client-side through React components, localStorage, and CSS custom properties.

---

## 1. ThemeConfig

### Description
Configuration for dynamic module-based theming including color mappings and typography settings.

### Structure

```typescript
interface ModuleTheme {
  id: 'ros2' | 'digitalTwin' | 'isaac' | 'vla' | 'default';
  name: string;
  primaryColor: string;      // Hex color
  accentColor: string;       // Hex color
  lightMode: {
    background: string;
    surface: string;
    text: string;
  };
  darkMode: {
    background: string;
    surface: string;
    text: string;
  };
}

interface ThemeConfig {
  modules: Record<string, ModuleTheme>;
  fonts: {
    display: string;         // "Orbitron"
    body: string;            // "Space Grotesk"
    fallback: string[];      // System font stack
  };
  animations: {
    enabled: boolean;
    reducedMotion: boolean;  // Detected from prefers-reduced-motion
    duration: {
      fast: number;          // 150ms
      normal: number;        // 300ms
      slow: number;          // 500ms
    };
  };
}
```

### Module Theme Values

| Module | Primary | Accent | Description |
|--------|---------|--------|-------------|
| ros2 | #FF6B35 | #FF8C5A | ROS 2 Fundamentals (orange) |
| digitalTwin | #4ECDC4 | #7EDDD6 | Digital Twin (teal) |
| isaac | #A855F7 | #C084FC | NVIDIA Isaac (purple) |
| vla | #FF0844 | #FF4D6D | Vision-Language-Action (red) |
| default | #2ECC71 | #22C55E | Default brand color (green) |

---

## 2. ProgressState

### Description
State for reading progress indicator and course completion tracking.

### Structure

```typescript
interface ReadingProgress {
  currentScroll: number;     // 0-100 percentage
  isComplete: boolean;       // True when reached bottom
  chapterId: string;         // Current chapter identifier
}

interface CourseProgress {
  [chapterId: string]: {
    completed: boolean;
    lastPosition: number;     // Scroll position
    completedAt?: number;     // Timestamp
  };
}

interface ProgressState {
  reading: ReadingProgress;
  course: CourseProgress;
  overallCompletion: number;  // 0-100 percentage
}
```

### LocalStorage Schema

```json
{
  "course-progress": {
    "/docs/intro": {
      "completed": true,
      "lastPosition": 100,
      "completedAt": 1736659200000
    },
    "/docs/week1/intro-physical-ai": {
      "completed": false,
      "lastPosition": 45
    }
  }
}
```

---

## 3. QuizState

### Description
Data structure for inline quiz components including questions, user answers, and scoring.

### Structure

```typescript
interface Question {
  id: string;
  type: 'multiple-choice' | 'true-false';
  question: string;
  options: string[];
  correctAnswer: number | string;
  explanation: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
}

interface QuizData {
  id: string;
  title: string;
  sectionId: string;        // Associated content section
  questions: Question[];
  passingScore: number;      // Default 70%
}

interface QuizAttempt {
  quizId: string;
  answers: Map<string, string>; // Question ID -> Selected answer
  score: number;
  passed: boolean;
  startedAt: number;
  completedAt?: number;
}

interface QuizState {
  current: QuizAttempt | null;
  history: QuizAttempt[];
  bestScores: Map<string, number>; // Quiz ID -> Best score
}
```

### LocalStorage Schema

```json
{
  "quiz-state": {
    "quiz-week1-basics": {
      "score": 80,
      "passed": true,
      "completedAt": 1736659200000
    }
  }
}
```

---

## 4. ThreeDModelConfig

### Description
Configuration for the 3D robot model including animation parameters and fallback settings.

### Structure

```typescript
interface ThreeDModelConfig {
  modelPath: string;         // Path to GLTF/GLB file
  scale: number;
  autoRotate: boolean;
  autoRotateSpeed: number;   // Radians per second
  enableZoom: boolean;
  enablePan: boolean;
  maxDistance: number;
  minDistance: number;
  fallback: {
    image: string;           // Static SVG fallback path
    alt: string;
  };
  performance: {
    low: {
      particleCount: number;
      shadowMap: boolean;
      antialiasing: boolean;
    };
    medium: {
      particleCount: number;
      shadowMap: boolean;
      antialiasing: boolean;
    };
    high: {
      particleCount: number;
      shadowMap: boolean;
      antialiasing: boolean;
    };
  };
}
```

---

## 5. VisualEffectsConfig

### Description
Configuration for visual effects including particle backgrounds and scroll animations.

### Structure

```typescript
interface ParticleConfig {
  enabled: boolean;
  count: number;
  size: { min: number; max: number };
  speed: { min: number; max: number };
  opacity: { min: number; max: number };
  color: string;
}

interface ScrollAnimationConfig {
  enabled: boolean;
  threshold: number;         // Intersection threshold (0-1)
  rootMargin: string;        // CSS margin for detection
  animations: {
    fadeIn: boolean;
    slideUp: boolean;
    scale: boolean;
  };
}

interface VisualEffectsConfig {
  particles: ParticleConfig;
  scroll: ScrollAnimationConfig;
  hover: {
    enabled: boolean;
    effects: ('glow' | 'magnetic' | 'tilt')[];
  };
}
```

---

## 6. Component State Models

### ChatIconState

```typescript
interface ChatIconState {
  isOpen: boolean;
  isPulsing: boolean;
  unreadCount: number;
}
```

### HeroAnimationState

```typescript
interface HeroAnimationState {
  phase: 'initial' | 'animating' | 'complete';
  currentStep: number;       // 0 to totalElements
  elements: {
    id: string;
    delay: number;            // Animation delay in ms
    animated: boolean;
  }[];
}
```

### ChapterNavigationState

```typescript
interface ChapterInfo {
  id: string;
  title: string;
  slug: string;
  prev?: ChapterInfo;
  next?: ChapterInfo;
}

interface ChapterNavigationState {
  current: ChapterInfo;
  isLoading: boolean;
}
```

---

## 7. CSS Custom Properties Schema

### Theme Variables

```css
:root {
  /* Module Colors (applied via data-module attribute) */
  --module-primary: #2ECC71;
  --module-accent: #22C55E;
  --module-bg-surface: #151e29;

  /* Typography */
  --font-display: 'Orbitron', system-ui, sans-serif;
  --font-body: 'Space Grotesk', system-ui, sans-serif;

  /* Spacing Scale */
  --space-xs: 0.25rem;    /* 4px */
  --space-sm: 0.5rem;     /* 8px */
  --space-md: 1rem;       /* 16px */
  --space-lg: 1.5rem;     /* 24px */
  --space-xl: 2rem;       /* 32px */
  --space-2xl: 3rem;      /* 48px */

  /* Animation Durations */
  --duration-fast: 150ms;
  --duration-normal: 300ms;
  --duration-slow: 500ms;

  /* Easing */
  --ease-out: cubic-bezier(0.215, 0.61, 0.355, 1);
  --ease-in-out: cubic-bezier(0.645, 0.045, 0.355, 1);
}
```

### Module-Specific Overrides

```css
[data-module="ros2"] {
  --module-primary: #FF6B35;
  --module-accent: #FF8C5A;
}

[data-module="digitalTwin"] {
  --module-primary: #4ECDC4;
  --module-accent: #7EDDD6;
}

[data-module="isaac"] {
  --module-primary: #A855F7;
  --module-accent: #C084FC;
}

[data-module="vla"] {
  --module-primary: #FF0844;
  --module-accent: #FF4D6D;
}
```

---

## 8. State Flow Diagrams

### Reading Progress Flow

```
Page Load → Detect Chapter → Load Saved Progress
                                  ↓
                         Scroll Event Listener
                                  ↓
                    Update Progress Bar (0-100%)
                                  ↓
                    Save to localStorage (debounced)
                                  ↓
                    Update Course Completion %
```

### Module Theme Flow

```
URL Change → Parse Path → Match Module Pattern
                              ↓
                      Set data-module attribute
                              ↓
                      CSS Variables Update
                              ↓
                      Components Re-render with New Theme
```

### Quiz Flow

```
User Loads Quiz → Read Quiz Data → Load Previous Attempt
                            ↓
                    User Selects Answer → Submit
                            ↓
                    Validate → Show Feedback → Update Score
                            ↓
                    Save to localStorage → Update Best Score
```

---

## 9. Validation Rules

### ThemeConfig
- `primaryColor` must be valid hex color (3 or 6 characters)
- `accentColor` must have minimum 4.5:1 contrast against white
- `animations.enabled` must be false if `prefers-reduced-motion` is true

### ProgressState
- `currentScroll` must be between 0-100
- `overallCompletion` must be calculated from completed chapters / total chapters

### QuizState
- `score` must be between 0-100
- `passed` is true if `score >= passingScore`

### ThreeDModelConfig
- `scale` must be positive
- `autoRotateSpeed` must be between -10 and 10 radians/second

---

## 10. State Persistence Summary

| Entity | Storage Method | Key | TTL |
|--------|----------------|-----|-----|
| Course Progress | localStorage | `course-progress` | Permanent |
| Quiz Scores | localStorage | `quiz-state` | Permanent |
| User Preferences | localStorage | `user-prefs` | Permanent |
| Reading Position | sessionStorage | `reading-position` | Session |
| Animation State | React State (memory) | N/A | Reset on navigation |
