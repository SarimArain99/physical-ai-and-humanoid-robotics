# Implementation Plan: UI/UX Improvements for AI-Native Textbook

**Branch**: `002-ui-improvements` | **Date**: 2026-01-12 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-ui-improvements/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature enhances the AI-Native Textbook on Physical AI & Humanoid Robotics with comprehensive UI/UX improvements across four priority tiers:

- **P1 (HIGH)**: Custom SVG chat icon, professional typography (Orbitron/Space Grotesk), hero animations, reading progress indicator
- **P2 (MEDIUM)**: Module-specific color theming, enhanced navigation with sidebar icons, page transitions, skeleton loaders
- **P3 (POLISH)**: Micro-interactions and advanced hover effects
- **P4 (ADVANCED)**: 3D robot illustration, interactive diagrams, inline quizzes, particle backgrounds

The implementation extends the existing Docusaurus frontend with new React components, CSS custom properties, and optional 3D graphics using Three.js.

## Technical Context

**Language/Version**: JavaScript (ES2022), React 18+, CSS3, TypeScript for type safety in new components
**Primary Dependencies**:
- Docusaurus 3.x (existing framework)
- Google Fonts API (Orbitron, Space Grotesk)
- Three.js (optional, for P4 3D robot)
- React Intersection Observer (for scroll animations)
- Framer Motion (optional, for advanced animations)
- Prism.js (existing, for code syntax highlighting)
**Storage**: N/A (client-side only, quiz state in localStorage)
**Testing**: Vitest for unit tests, Playwright for E2E visual regression tests
**Target Platform**: Modern web browsers (Chrome 100+, Firefox 100+, Safari 15+, Edge 100+)
**Project Type**: Web (Docusaurus static site generator)
**Performance Goals**:
  - Homepage animations complete within 1 second
  - Page transitions complete within 300ms
  - 3D model maintains 30+ FPS during interaction
  - Scroll animations complete before element fully visible
**Constraints**:
  - Must support prefers-reduced-motion accessibility
  - Must provide WebGL fallback for 3D content
  - Must meet WCAG AA contrast requirements
  - Must gracefully degrade on low-end devices
**Scale/Scope**: 9 user stories, 27 functional requirements, 4 priority tiers (P1-P4), approximately 15-20 new components

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle 1: AI-Native Pedagogy
✅ **PASS** - UI improvements enhance the AI-augmented learning experience. Reading progress indicators, interactive diagrams, and inline quizzes directly support RAG-based learning.

### Principle 2: Technical Accuracy
✅ **PASS** - Implementation uses established web standards and libraries. Technical choices (Docusaurus, React, Three.js) are well-documented and industry-standard.

### Principle 3: Accessibility & Inclusivity
✅ **PASS** - All requirements include accessibility considerations:
- FR-011: Respects prefers-reduced-motion
- FR-012: WCAG AA contrast compliance
- FR-013: Font fallbacks
- SC-008: No animations for reduced-motion users
- 3D model has WebGL fallback (FR-018)

### Principle 4: Open & Reproducible
✅ **PASS** - All code will be open-source, using standard Docusaurus extension patterns. No proprietary dependencies.

### Principle 5: Interactive Learning
✅ **PASS** - Feature directly enhances interactive learning with:
- Interactive 3D robot model
- Interactive concept diagrams
- Inline knowledge checks
- Enhanced chat interface

### Quality Standards Compliance
✅ **PASS** - All technical implementation standards met:
- Uses Docusaurus (existing framework)
- No new backend dependencies required
- Quiz state stored locally (localStorage)
- Visual effects gracefully degrade

**Overall Status**: ✅ **ALL GATES PASSED** - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/002-ui-improvements/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - Technical decisions
├── data-model.md        # Phase 1 output - Component entities
├── quickstart.md        # Phase 1 output - Developer guide
├── contracts/           # Phase 1 output - Component interfaces
│   ├── components.md    # Component specifications
│   └── styles.md        # Design system contracts
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   │   ├── index.js          # Existing - to be enhanced
│   │   │   ├── ChatIcon.js       # NEW: Custom SVG robot icon
│   │   │   └── CodeHighlight.js  # NEW: Syntax highlighting
│   │   ├── ReadingProgress/     # NEW: Reading progress indicator
│   │   ├── HeroAnimation/       # NEW: Staggered hero animations
│   │   ├── WeekCards/           # NEW: Enhanced week cards
│   │   ├── ChapterNavigation/   # NEW: Previous/Next navigation
│   │   ├── SidebarIcons/        # NEW: Module icons for sidebar
│   │   ├── PageTransition/      # NEW: Page transition wrapper
│   │   ├── SkeletonLoader/      # NEW: Loading skeletons
│   │   ├── ThreeDRobot/         # NEW: 3D robot model (P4)
│   │   ├── InteractiveDiagram/  # NEW: ROS 2 diagrams (P4)
│   │   ├── QuizComponent/       # NEW: Inline quizzes (P4)
│   │   └── VisualEffects/        # NEW: Particle backgrounds (P4)
│   ├── css/
│   │   ├── custom.css           # Existing - to be enhanced
│   │   ├── theme.css            # NEW: Module-specific theming
│   │   └── animations.css       # NEW: Animation definitions
│   ├── theme/
│   │   ├── Layout/              # Existing - may need enhancement
│   │   └── Typography/          # NEW: Font configuration
│   └── utils/
│       ├── moduleDetection.js   # NEW: Detect current module from URL
│       └── performance.js        # NEW: Device capability detection
├── static/
│   └── models/                  # NEW: 3D model assets (P4)
└── docusaurus.config.ts         # Existing - to add fonts

contracts/                          # Design contracts for this feature
├── component-specs.md             # Component interface definitions
└── design-tokens.md               # Color, typography, spacing standards
```

**Structure Decision**: Web application structure (Option 2) - This is a frontend-only enhancement to the existing Docusaurus site. No backend changes required as all state is managed client-side.

## Complexity Tracking

> **No violations requiring justification** - All UI improvements are within standard frontend development complexity for a Docusaurus site.

## Phase 0: Research & Technical Decisions

See [research.md](./research.md) for detailed findings on:

1. **Font Selection**: Orbitron (display) and Space Grotesk (body) - chosen for their technical/robotic aesthetic while maintaining readability
2. **3D Graphics**: Three.js selected with fallback image strategy
3. **Animation Strategy**: CSS animations for performance, Intersection Observer for scroll triggers
4. **Module Detection**: URL path-based detection for dynamic theming
5. **Accessibility**: Progressive enhancement with prefers-reduced-motion detection
6. **Code Highlighting**: Prism.js extension for chat responses
7. **Icon Strategy**: Inline SVGs for reliability and performance

## Phase 1: Design & Contracts

### Data Model

See [data-model.md](./data-model.md) for:

- **ThemeConfig**: Module color mappings, font configurations
- **ProgressState**: Reading position, completion tracking
- **QuizState**: Question data, user answers, scoring (localStorage schema)
- **3DModelConfig**: Model settings, animation parameters, fallback options

### Component Contracts

See [contracts/components.md](./contracts/components.md) for:

- **IReadingProgress**: Props, events, behavior contract
- **IChatIcon**: SVG icon specification, animation contract
- **IHeroAnimation**: Staggered animation timing, element sequencing
- **IChapterNavigation**: Previous/Next link generation
- **IModuleTheme**: Dynamic theme application contract
- **IThreeDRobot**: 3D model interaction contract
- **IQuiz**: Question format, feedback, scoring contract
- **IVisualEffects**: Performance-aware animation contract

### Design Tokens

See [contracts/styles.md](./contracts/styles.md) for:

- **Module Colors**:
  - ROS 2: `#FF6B35` (orange)
  - Digital Twin: `#4ECDC4` (teal)
  - NVIDIA Isaac: `#A855F7` (purple)
  - VLA: `#FF0844` (red)
- **Typography**: Font sizes, weights, line heights
- **Spacing**: Consistent spacing scale
- **Animation**: Duration curves, easing functions
- **Breakpoints**: Mobile-first responsive design

## Phase 2: Implementation Phases

### Phase 2.1: P1 Foundation (HIGH Priority)

**Goal**: Deliver quick-win visual improvements

**Tasks**:
1. Add Google Fonts integration (Orbitron, Space Grotesk)
2. Create custom SVG robot icon for chat toggle
3. Implement reading progress indicator component
4. Add hero section staggered animations
5. Enhance week cards with hover effects

**Success Criteria**: SC-001, SC-003, SC-006

### Phase 2.2: P2 Enhancement (MEDIUM Priority)

**Goal**: Add module theming and navigation improvements

**Tasks**:
1. Implement module-specific color theming system
2. Add icons to sidebar navigation
3. Create chapter navigation component
4. Implement page transition animations
5. Add skeleton loaders for async operations

**Success Criteria**: SC-002, SC-004, SC-009, SC-010

### Phase 2.3: P3 Polish (POLISH Priority)

**Goal**: Refine micro-interactions

**Tasks**:
1. Add advanced hover effects
2. Implement micro-interactions for all interactive elements
3. Add loading state transitions

**Success Criteria**: SC-005

### Phase 2.4: P4 Advanced (ADVANCED Priority)

**Goal**: Add "wow factor" features

**Tasks**:
1. Implement Three.js 3D robot model with WebGL fallback
2. Create interactive ROS 2 diagrams
3. Build quiz component with localStorage persistence
4. Add particle background with performance detection

**Success Criteria**: SC-011 through SC-020

## Dependencies & Integration Points

1. **Existing Docusaurus Config**: Extend with custom CSS, font imports
2. **ChatWidget Component**: Enhance with SVG icon and code highlighting
3. **Layout Component**: May need wrapper for page transitions
4. **Markdown Content**: Add quiz embed syntax, diagram placeholders
5. **URL Routing**: Module detection from path for dynamic theming

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Font loading delay | Preload fonts, provide system font fallback |
| WebGL not supported | Fallback to static image for 3D model |
| Performance on low-end devices | Progressive enhancement, performance detection |
| Animation accessibility | Respect prefers-reduced-motion |
| Browser compatibility | Test on target browsers, provide polyfills if needed |

## Next Steps

1. ✅ Constitution check passed
2. ✅ Research completed (research.md)
3. ✅ Data model defined (data-model.md)
4. ✅ Contracts specified (contracts/)
5. ✅ Quickstart guide created (quickstart.md)
6. ⏭️ Run `/sp.tasks` to generate actionable task breakdown
