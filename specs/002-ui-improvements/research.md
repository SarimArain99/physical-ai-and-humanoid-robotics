# Research: UI/UX Improvements Technical Decisions

**Feature**: 002-ui-improvements
**Date**: 2026-01-12
**Status**: Complete

## Overview

This document captures the research and technical decisions for implementing UI/UX improvements to the AI-Native Textbook. All decisions prioritize performance, accessibility, and maintainability while delivering a distinctive "Physical AI" aesthetic.

---

## 1. Font Selection (FR-002, FR-013)

### Decision
**Orbitron** (display/headings) + **Space Grotesk** (body text)

### Rationale
- **Orbitron**: Geometric, futuristic sans-serif that conveys technology and robotics. Wide character set includes technical symbols.
- **Space Grotesk**: Highly readable geometric sans-serif with excellent performance. Designed for UI use.
- Both fonts are available via Google Fonts CDN with consistent uptime
- WOFF2 format provides excellent compression (~20KB for full family)

### Alternatives Considered
| Font | Pros | Cons | Rejected Because |
|------|------|-------|------------------|
| Rajdhani | Technical, narrow | Limited weight options | Less distinctive than Orbitron |
| Inter | Highly readable | Overused, generic | Doesn't convey "Physical AI" theme |
| Roboto Mono | Technical aesthetic | Too monospace for body | Poor readability for long-form content |
| Exo 2 | Geometric, futuristic | Slightly heavier | Orbitron more distinctive |

### Implementation
```typescript
// docusaurus.config.ts
const config = {
  themeConfig: {
    customCss: [
      require.resolve('./src/css/fonts.css'),
    ],
  },
  // Preconnect for performance
  scripts: [
    {
      src: 'https://fonts.googleapis.com',
      async: true,
    },
    {
      src: 'https://fonts.gstatic.com',
      async: true,
    },
  ],
};
```

```css
/* src/css/fonts.css */
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;500;600;700;800;900&family=Space+Grotesk:wght@300;400;500;600;700&display=swap');

:root {
  --font-display: 'Orbitron', sans-serif;
  --font-body: 'Space Grotesk', system-ui, -apple-system, sans-serif;
}

/* Fallback chain ensures graceful degradation */
```

---

## 2. 3D Graphics Strategy (FR-016, FR-017, FR-018)

### Decision
**Three.js** with **WebGL detection** and **fallback image**

### Rationale
- Three.js is the most mature 3D library for web (100+ MB download base, extensive documentation)
- Built-in WebGL capability detection via `renderer.capabilities.isWebGL2`
- Lightweight model format: GLTF/GLB for efficient loading
- Fallback strategy: static SVG illustration when WebGL unavailable

### Alternatives Considered
| Library | Pros | Cons | Rejected Because |
|---------|------|-------|------------------|
| Babylon.js | More features | Heavier bundle (3MB+) | Overkill for simple rotation |
| react-three-fiber | React integration | Additional abstraction layer | Direct Three.js simpler here |
| pure WebGL | Best performance | Very verbose code | Development time too high |

### Implementation Plan

```javascript
// src/components/ThreeDRobot/index.js
import * as THREE from 'three';

const RobotModel = () => {
  const canvasRef = useRef(null);
  const [webGLSupported, setWebGLSupported] = useState(true);

  useEffect(() => {
    // Capability detection
    const canvas = document.createElement('canvas');
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');

    if (!gl) {
      setWebGLSupported(false);
      return;
    }

    // Initialize Three.js scene
    // Load model, setup rotation, etc.
  }, []);

  if (!webGLSupported) {
    return <FallbackSVG />;
  }

  return <canvas ref={canvasRef} />;
};
```

### Model Considerations
- Use simplified GLTF model (<500KB compressed)
- Auto-rotate only when idle (pause on interaction)
- LOD (Level of Detail) for mobile devices
- Progressive loading: show placeholder first

---

## 3. Animation Strategy (FR-005, FR-007, FR-011)

### Decision
**CSS animations** for simple effects + **Intersection Observer** for scroll triggers

### Rationale
- CSS animations run on compositor thread (better performance than JS)
- `will-change` property hints browser for optimization
- Intersection Observer API provides efficient viewport detection
- No additional library dependencies for core animations

### Alternatives Considered
| Approach | Pros | Cons | Rejected Because |
|----------|------|-------|------------------|
| Framer Motion | Declarative API | Additional 40KB bundle | CSS sufficient for most needs |
| GSAP | Powerful timeline control | Commercial license for many uses | Overkill for our use case |
| Web Animations API | Native, performant | Less browser support | CSS provides better fallback |

### Performance Considerations
```css
/* Use GPU acceleration */
.animated-element {
  will-change: transform, opacity;
  transform: translateZ(0); /* Force GPU layer */
}

/* Respect reduced motion preference */
@media (prefers-reduced-motion: reduce) {
  .animated-element {
    animation: none !important;
    transition: none !important;
  }
}
```

---

## 4. Module Detection for Dynamic Theming (FR-004)

### Decision
**URL path-based detection** with **data attribute** fallback

### Rationale
- Docusaurus routes follow predictable pattern: `/docs/week1/`, `/docs/week2/`, etc.
- Path-based detection is simple and reliable
- Data attribute allows manual override for special pages

### Implementation

```javascript
// src/utils/moduleDetection.js
const MODULE_CONFIG = {
  ros2: {
    pattern: /^\/docs\/(week[1-5]|intro)/,
    color: '#FF6B35',
    name: 'ROS 2 Fundamentals',
  },
  digitalTwin: {
    pattern: /^\/docs\/(week[6-7])/,
    color: '#4ECDC4',
    name: 'Digital Twin',
  },
  isaac: {
    pattern: /^\/docs\/(week[8-10])/,
    color: '#A855F7',
    name: 'NVIDIA Isaac',
  },
  vla: {
    pattern: /^\/docs\/(week[11-13])/,
    color: '#FF0844',
    name: 'Vision-Language-Action',
  },
};

export function detectModule(path) {
  for (const [key, config] of Object.entries(MODULE_CONFIG)) {
    if (config.pattern.test(path)) {
      return { key, ...config };
    }
  }
  return null; // Default theme
}
```

---

## 5. Accessibility Strategy (FR-011, FR-012, FR-013)

### Decision
**Progressive enhancement** with **feature detection**

### Requirements Coverage
- **FR-011** (prefers-reduced-motion): CSS media query detection
- **FR-012** (WCAG AA contrast): Automated testing via axe-core
- **FR-013** (font fallback): CSS font-stack with system fonts

### Implementation

```javascript
// src/utils/accessibility.js
export const prefersReducedMotion = () =>
  window.matchMedia('(prefers-reduced-motion: reduce)').matches;

export const hasWebGL = () => {
  const canvas = document.createElement('canvas');
  return !!(canvas.getContext('webgl2') || canvas.getContext('webgl'));
};

export const getContrastColor = (hexColor) => {
  // Calculate luminance and return black/white
  // Ensures WCAG AA compliance
};
```

### Testing Strategy
- axe-core DevTools extension during development
- Playwright accessibility assertions in CI
- Manual keyboard navigation testing

---

## 6. Code Highlighting in Chat (FR-010)

### Decision
**Prism.js** (already bundled with Docusaurus)

### Rationale
- Prism.js is already included in Docusaurus for code blocks
- Zero additional bundle cost
- Extensive language support
- Themeable via CSS variables

### Implementation

```javascript
// src/components/ChatWidget/CodeHighlight.js
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { vscDarkPlus } from 'react-syntax-highlighter/dist/esm/styles/prism';

const CodeBlock = ({ code, language }) => (
  <SyntaxHighlighter
    language={language || 'text'}
    style={vscDarkPlus}
    customStyle={{ borderRadius: '8px', fontSize: '14px' }}
    showLineNumbers
  >
    {code}
  </SyntaxHighlighter>
);
```

---

## 7. Icon Strategy (FR-001, FR-009)

### Decision
**Inline SVG** components for critical icons + **Lucide React** for system icons

### Rationale
- Inline SVG: No network request, immediate rendering, full styling control via CSS
- Lucide React: Lightweight (~1KB per icon), tree-shakeable
- Consistent stroke width (2px) across all icons

### Icon Inventory

| Component | Icon | Source |
|-----------|------|--------|
| Chat toggle | Custom robot SVG | Inline |
| ROS 2 module | Gear/Cog | Lucide |
| Digital Twin | Box/Cube | Lucide |
| NVIDIA Isaac | Cpu | Lucide |
| VLA | Eye | Lucide |

---

## 8. Performance Detection (P4 - SC-017, SC-020)

### Decision
**Navigator.hardwareConcurrency** + **DeviceMemory API** with **graceful degradation**

### Rationale
- Native browser APIs, no external dependencies
- Three-tier system: High/Medium/Low quality
- Automatic adjustment based on device capability

### Implementation

```javascript
// src/utils/performance.js
export const getPerformanceTier = () => {
  const cores = navigator.hardwareConcurrency || 2;
  const memory = navigator.deviceMemory || 4;

  if (cores >= 8 && memory >= 8) return 'high';
  if (cores >= 4 && memory >= 4) return 'medium';
  return 'low';
};

export const shouldReduceEffects = () => {
  return getPerformanceTier() === 'low' || prefersReducedMotion();
};
```

---

## 9. Quiz Storage Strategy (P4 - FR-022, FR-023, FR-024)

### Decision
**localStorage** with **JSON serialization**

### Rationale
- No backend dependency required
- Persistent across sessions
- Simple key-value structure
- Fallback to session storage if localStorage unavailable

### Schema

```typescript
interface QuizAnswer {
  questionId: string;
  selected: string;
  correct: boolean;
  timestamp: number;
}

interface QuizProgress {
  [quizId: string]: {
    answers: QuizAnswer[];
    score: number;
    completedAt: number;
  };
}
```

---

## 10. Particle Background Strategy (P4 - FR-025)

### Decision
**HTML5 Canvas** with **requestAnimationFrame** loop

### Rationale
- Canvas more performant than DOM nodes for many particles
- requestAnimationFrame syncs with display refresh rate
- Can pause on visibility change (Page Visibility API)

### Performance Optimization
```javascript
const PARTICLE_COUNTS = {
  high: 100,
  medium: 50,
  low: 0, // Disabled on low-end devices
};

const getParticleCount = () => {
  const tier = getPerformanceTier();
  return PARTICLE_COUNTS[tier] || 0;
};
```

---

## Summary of Decisions

| Area | Technology | Justification |
|------|-----------|---------------|
| Fonts | Orbitron + Space Grotesk | Technical aesthetic, readable |
| 3D Graphics | Three.js + WebGL fallback | Mature, performant |
| Animations | CSS + Intersection Observer | Native performance |
| Module Detection | URL path-based | Simple, reliable |
| Accessibility | Progressive enhancement | WCAG AA compliant |
| Code Highlighting | Prism.js (existing) | Zero additional cost |
| Icons | Inline SVG + Lucide React | Performant, consistent |
| Performance | Navigator APIs | Native detection |
| Quiz Storage | localStorage | No backend needed |
| Particles | Canvas + rAF | GPU-accelerated |

---

## References

1. Google Fonts: https://fonts.google.com/
2. Three.js Documentation: https://threejs.org/docs/
3. Intersection Observer API: https://developer.mozilla.org/en-US/docs/Web/API/Intersection_Observer_API
4. WCAG 2.1 Guidelines: https://www.w3.org/WAI/WCAG21/quickref/
5. Prism.js: https://prismjs.com/
6. Lucide Icons: https://lucide.dev/
