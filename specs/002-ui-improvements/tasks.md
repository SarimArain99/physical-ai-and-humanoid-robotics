# Tasks: UI/UX Improvements for AI-Native Textbook

**Feature**: 002-ui-improvements
**Branch**: `002-ui-improvements`
**Input**: Design documents from `/specs/002-ui-improvements/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are optional for this feature. E2E visual regression tests using Playwright are recommended but not required for initial delivery.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `frontend/src/` for React components, CSS, utilities
- **Config**: `frontend/docusaurus.config.ts` for Docusaurus configuration
- **Static**: `frontend/static/` for static assets (images, models)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [X] T001 Install frontend dependencies: three, @types/three, react-intersection-observer in frontend/
- [X] T002 [P] Create component directory structure in frontend/src/components/ (ReadingProgress, HeroAnimation, ChapterNavigation, etc.)
- [X] T003 [P] Create CSS directory in frontend/src/css/ for theme and animation files
- [X] T004 [P] Create utilities directory in frontend/src/utils/ for module detection and performance utilities
- [X] T005 Create static assets directory in frontend/static/models/ for 3D model files

**Checkpoint**: Project structure ready, dependencies installed

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core theming infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Create Google Fonts integration in frontend/src/css/fonts.css (Orbitron, Space Grotesk with fallbacks)
- [X] T007 [P] Create design tokens CSS in frontend/src/css/theme.css (module colors, spacing, typography scale from contracts/styles.md)
- [X] T008 [P] Create animation definitions in frontend/src/css/animations.css (keyframes for fadeIn, slideUp, scale, pulse, shimmer)
- [X] T009 Create accessibility overrides in frontend/src/css/animations.css (@media prefers-reduced-motion)
- [X] T010 Create module detection utility in frontend/src/utils/moduleDetection.js (URL path parsing for ros2, digitalTwin, isaac, vla)
- [X] T011 Create performance detection utility in frontend/src/utils/performance.js (hardware detection, graceful degradation)
- [X] T012 Create ModuleThemeProvider component in frontend/src/components/ModuleThemeProvider/index.js
- [X] T013 Integrate fonts and custom CSS in frontend/docusaurus.config.ts (add customCss array)
- [X] T014 Swizzle Layout component to wrap with ModuleThemeProvider in frontend/src/theme/Layout/index.js

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Enhanced Visual Identity (Priority: P1) 🎯 MVP

**Goal**: Deliver distinctive, professional visual design with custom typography, hero animations, and week card hover effects

**Independent Test**: Visit the homepage and observe the enhanced hero section with custom animations, Orbitron/Space Grotesk typography, and week cards with hover effects

### Implementation for User Story 1

- [X] T015 [P] [US1] Create HeroAnimation component in frontend/src/components/HeroAnimation/index.js with staggered reveal logic
- [X] T016 [P] [US1] Create HeroAnimation styles in frontend/src/components/HeroAnimation/styles.css (slideUp, fadeIn animations)
- [X] T017 [P] [US1] Create WeekCards component in frontend/src/components/WeekCards/index.js with hover state handling
- [X] T018 [P] [US1] Create WeekCards styles in frontend/src/components/WeekCards/styles.css (hover effects, card transitions)
- [X] T019 [US1] Integrate HeroAnimation into homepage hero section (modify homepage React component or MDX)
- [X] T020 [US1] Integrate WeekCards component on homepage (replace existing week cards)
- [X] T021 [US1] Verify typography loads correctly on homepage (Orbitron headings, Space Grotesk body)
- [X] T022 [US1] Add accessibility attributes to animated elements (aria-live, reduced-motion checks)

**Checkpoint**: User Story 1 complete - homepage has distinctive visual design with animations and hover effects

---

## Phase 4: User Story 2 - Professional Chat Interface (Priority: P1)

**Goal**: Replace emoji chat button with custom SVG robot icon and polish chat window design

**Independent Test**: Open the chat widget and observe the custom SVG robot icon (not emoji), pulsing glow effect, and polished chat window

### Implementation for User Story 2

- [X] T023 [P] [US2] Create ChatIcon component in frontend/src/components/ChatWidget/ChatIcon.js with SVG robot illustration
- [X] T024 [P] [US2] Create ChatIcon styles in frontend/src/components/ChatWidget/ChatIcon.css (pulsing glow animation, hover states)
- [X] T025 [P] [US2] Create CodeHighlight component in frontend/src/components/ChatWidget/CodeHighlight.js (syntax highlighting wrapper)
- [X] T026 [US2] Update ChatWidget index to use ChatIcon component in frontend/src/components/ChatWidget/index.js (replace emoji)
- [X] T027 [US2] Update ChatWidget styles for polished appearance in frontend/src/components/ChatWidget/styles.css
- [X] T028 [US2] Add copy button to code blocks in chat responses (CodeHighlight component)
- [X] T029 [US2] Add typing indicator animation to chat window in frontend/src/components/ChatWidget/styles.css
- [X] T030 [US2] Verify accessibility: keyboard navigation, aria-labels on chat icon

**Checkpoint**: User Story 2 complete - chat interface uses custom SVG icon with polished design

---

## Phase 5: User Story 3 - Module-Specific Visual Theming (Priority: P2)

**Goal**: Apply dynamic accent colors based on course module (ROS 2: orange, Digital Twin: teal, Isaac: purple, VLA: red)

**Independent Test**: Navigate to different week pages and observe module-specific accent colors

### Implementation for User Story 3

- [X] T031 [P] [US3] Create module-specific color overrides in frontend/src/css/theme.css ([data-module="ros2"], [data-module="digitalTwin"], etc.)
- [X] T032 [P] [US3] Update ModuleThemeProvider to set CSS variables in frontend/src/components/ModuleThemeProvider/index.js
- [X] T033 [P] [US3] Create module indicator component in frontend/src/components/ModuleIndicator/index.js (shows current module name/color)
- [X] T034 [US3] Add module-specific styles for buttons, links, borders in frontend/src/css/theme.css
- [X] T035 [US3] Test module color transitions when navigating between weeks (smooth 300ms transition)
- [X] T036 [US3] Verify WCAG AA contrast for all module color combinations

**Checkpoint**: User Story 3 complete - each module has distinctive accent color theming

---

## Phase 6: User Story 4 - Enhanced Navigation and Progress Tracking (Priority: P2)

**Goal**: Add reading progress indicator and chapter navigation links

**Independent Test**: Open any content page and observe the reading progress bar at top, Previous/Next links at bottom

### Implementation for User Story 4

- [X] T037 [P] [US4] Create ReadingProgress component in frontend/src/components/ReadingProgress/index.js (scroll tracking)
- [X] T038 [P] [US4] Create ReadingProgress styles in frontend/src/components/ReadingProgress/styles.css (fixed top bar, module color fill)
- [X] T039 [P] [US4] Create ChapterNavigation component in frontend/src/components/ChapterNavigation/index.js (prev/next links)
- [X] T040 [P] [US4] Create ChapterNavigation styles in frontend/src/components/ChapterNavigation/styles.css
- [X] T041 [P] [US4] Create course progress storage utility in frontend/src/utils/progressStorage.js (localStorage for completion)
- [X] T042 [US4] Integrate ReadingProgress into doc pages (swizzle DocPage or use client module)
- [X] T043 [US4] Integrate ChapterNavigation at bottom of doc pages
- [X] T044 [US4] Add completion tracking to ReadingProgress (save to localStorage on 100% scroll)

**Checkpoint**: User Story 4 complete - reading progress and chapter navigation functional

---

## Phase 7: User Story 5 - Loading States and Micro-interactions (Priority: P3)

**Goal**: Add page transitions, skeleton loaders, and hover feedback for all interactive elements

**Independent Test**: Navigate pages, trigger loading states, and observe smooth transitions and skeleton loaders

### Implementation for User Story 5

- [X] T045 [P] [US5] Create PageTransition wrapper component in frontend/src/components/PageTransition/index.js
- [X] T046 [P] [US5] Create PageTransition styles in frontend/src/components/PageTransition/styles.css (fade, slide, scale transitions)
- [X] T047 [P] [US5] Create SkeletonLoader component in frontend/src/components/SkeletonLoader/index.js
- [X] T048 [P] [US5] Create SkeletonLoader styles in frontend/src/components/SkeletonLoader/styles.css (shimmer animation)
- [X] T049 [P] [US5] Create micro-interaction styles in frontend/src/css/theme.css (button hover, link hover, focus states)
- [X] T050 [US5] Integrate PageTransition into Layout wrapper in frontend/src/theme/Layout/index.js
- [X] T051 [US5] Add skeleton loaders to ChatWidget during API responses (typing indicator exists)
- [X] T052 [US5] Add hover effects to all interactive elements (buttons, cards, navigation)
- [X] T053 [US5] Verify all animations respect prefers-reduced-motion

**Checkpoint**: User Story 5 complete - smooth transitions, loading states, and micro-interactions implemented

---

## Phase 8: User Story 6 - Interactive 3D Robot Illustration (Priority: P4)

**Goal**: Add interactive 3D humanoid robot model on homepage with rotation controls

**Independent Test**: Visit homepage and observe 3D robot model with drag-to-rotate interaction

### Implementation for User Story 6

- [X] T054 [P] [US6] Create ThreeDRobot component in frontend/src/components/ThreeDRobot/index.js (Three.js integration)
- [X] T055 [P] [US6] Create ThreeDRobot styles in frontend/src/components/ThreeDRobot/styles.css (canvas sizing, loading state)
- [X] T056 [P] [US6] Create 3D model fallback SVG in frontend/src/components/ThreeDRobot/FallbackSVG.js (for no WebGL)
- [X] T057 [US6] Add WebGL detection and fallback logic in frontend/src/components/ThreeDRobot/index.js
- [X] T058 [US6] Add auto-rotate animation in frontend/src/components/ThreeDRobot/index.js
- [X] T059 [US6] Add mouse/touch drag controls in frontend/src/components/ThreeDRobot/index.js
- [X] T060 [US6] Integrate ThreeDRobot into homepage hero section
- [X] T061 [US6] Test performance on mid-range devices (target 30+ FPS)

**Checkpoint**: User Story 6 complete - interactive 3D robot model functional with WebGL fallback

---

## Phase 9: User Story 7 - Interactive Concept Diagrams (Priority: P4)

**Goal**: Add interactive ROS 2 diagrams with clickable nodes and animations

**Independent Test**: View pages with diagrams and verify hover tooltips and clickable elements

### Implementation for User Story 7

- [X] T062 [P] [US7] Create InteractiveDiagram component in frontend/src/components/InteractiveDiagram/index.js
- [X] T063 [P] [US7] Create InteractiveDiagram styles in frontend/src/components/InteractiveDiagram/styles.css (node hover, tooltip styles)
- [X] T064 [P] [US7] Create diagram node data structure in frontend/src/data/diagrams.js (ROS 2 architecture definitions)
- [X] T065 [US7] Add SVG rendering for diagram nodes in frontend/src/components/InteractiveDiagram/index.js
- [X] T066 [US7] Add tooltip component for node information in frontend/src/components/InteractiveDiagram/Tooltip.js
- [X] T067 [US7] Add data flow animation in frontend/src/components/InteractiveDiagram/index.js
- [X] T068 [US7] Create Markdown/MDX plugin or directive for embedding diagrams in content
- [X] T069 [US7] Add keyboard navigation for diagram accessibility

**Checkpoint**: User Story 7 complete - interactive diagrams with hover tooltips and animations

---

## Phase 10: User Story 8 - Inline Knowledge Checks (Priority: P4)

**Goal**: Add quiz components after content sections with immediate feedback

**Independent Test**: Read section with embedded quiz, submit answers, verify feedback and scoring

### Implementation for User Story 8

- [X] T070 [P] [US8] Create Quiz component in frontend/src/components/Quiz/index.js
- [X] T071 [P] [US8] Create Quiz styles in frontend/src/components/Quiz/styles.css (question card, feedback states)
- [X] T072 [P] [US8] Create quiz storage utility in frontend/src/utils/quizStorage.js (localStorage for scores)
- [X] T073 [P] [US8] Create quiz data definitions in frontend/src/data/quizzes.js (questions by section)
- [X] T074 [US8] Implement answer selection and validation logic in frontend/src/components/Quiz/index.js
- [X] T075 [US8] Implement feedback display (correct/incorrect with explanations) in frontend/src/components/Quiz/index.js
- [X] T076 [US8] Implement scoring and review functionality in frontend/src/components/Quiz/index.js
- [X] T077 [US8] Create Markdown/MDX component for embedding quizzes in content
- [X] T078 [US8] Add accessibility: ARIA labels, keyboard navigation, screen reader announcements

**Checkpoint**: User Story 8 complete - inline quizzes with feedback and scoring functional

---

## Phase 11: User Story 9 - Advanced Visual Effects (Priority: P4)

**Goal**: Add particle backgrounds, scroll-triggered animations, and advanced hover effects

**Independent Test**: Scroll through pages and observe background particles, element animations, and hover effects

### Implementation for User Story 9

- [X] T079 [P] [US9] Create VisualEffects provider component in frontend/src/components/VisualEffects/index.js
- [X] T080 [P] [US9] Create particle background component in frontend/src/components/VisualEffects/ParticleBackground.js
- [X] T081 [P] [US9] Create scroll animation observer in frontend/src/components/VisualEffects/ScrollAnimations.js
- [X] T082 [P] [US9] Create advanced hover effects in frontend/src/css/theme.css (glow, magnetic, tilt effects)
- [X] T083 [US9] Implement canvas-based particle rendering in frontend/src/components/VisualEffects/ParticleBackground.js
- [X] T084 [US9] Implement Intersection Observer for scroll animations in frontend/src/components/VisualEffects/ScrollAnimations.js
- [X] T085 [US9] Add performance-based particle count adjustment in frontend/src/components/VisualEffects/ParticleBackground.js
- [X] T086 [US9] Integrate VisualEffects provider into Layout wrapper
- [X] T087 [US9] Verify effects respect prefers-reduced-motion

**Checkpoint**: User Story 9 complete - particle backgrounds and scroll animations implemented

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, testing, and documentation

- [X] T088 [P] Run accessibility audit with axe DevTools or similar (verify WCAG AA compliance)
- [X] T089 [P] Test cross-browser compatibility (Chrome, Firefox, Safari, Edge)
- [X] T090 [P] Test responsive design on mobile devices (320px+ viewport)
- [X] T091 [P] Update quickstart.md with any additional setup steps discovered during implementation
- [X] T092 Performance optimization: lazy load 3D models, optimize images, minify CSS
- [X] T093 Add visual regression tests (Playwright screenshots for key pages)
- [X] T094 Code cleanup: remove unused imports, consolidate duplicate styles
- [X] T095 Run quickstart.md validation to ensure all setup steps work

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-11)**: All depend on Foundational phase completion
- **Polish (Phase 12)**: Depends on all desired user stories being complete

### User Story Dependencies

| User Story | Priority | Can Start After | Depends On |
|------------|----------|-----------------|------------|
| US1 - Enhanced Visual Identity | P1 | Foundational (Phase 2) | No other stories |
| US2 - Professional Chat Interface | P1 | Foundational (Phase 2) | No other stories |
| US3 - Module Theming | P2 | Foundational (Phase 2) | No other stories |
| US4 - Navigation & Progress | P2 | Foundational (Phase 2) | No other stories |
| US5 - Loading States | P3 | Foundational (Phase 2) | No other stories |
| US6 - 3D Robot | P4 | Foundational (Phase 2) | No other stories |
| US7 - Interactive Diagrams | P4 | Foundational (Phase 2) | No other stories |
| US8 - Quizzes | P4 | Foundational (Phase 2) | No other stories |
| US9 - Advanced Effects | P4 | Foundational (Phase 2) | No other stories |

**Key Insight**: All user stories are independent after the foundational phase and can be implemented in any order or in parallel.

### Within Each User Story

- Tasks marked [P] within a story can run in parallel
- Tasks without [P] have dependencies within that story
- Integration tasks typically depend on component creation tasks

### Parallel Opportunities

**Setup Phase (Phase 1)**:
```bash
# All can run in parallel:
T002: Create component directory structure
T003: Create CSS directory
T004: Create utilities directory
```

**Foundational Phase (Phase 2)**:
```bash
# CSS files can run in parallel:
T007: Create design tokens CSS
T008: Create animation definitions CSS
```

**User Story 1 (Enhanced Visual Identity)**:
```bash
# Component creation can run in parallel:
T015: Create HeroAnimation component
T016: Create HeroAnimation styles
T017: Create WeekCards component
T018: Create WeekCards styles
```

**User Story 2 (Professional Chat Interface)**:
```bash
# Component creation can run in parallel:
T023: Create ChatIcon component
T024: Create ChatIcon styles
T025: Create CodeHighlight component
```

**User Story 3 (Module Theming)**:
```bash
# Tasks can run in parallel:
T031: Create module-specific color overrides
T032: Update ModuleThemeProvider
T033: Create module indicator component
T034: Add module-specific styles
```

---

## Implementation Strategy

### MVP First (P1 User Stories Only)

For quick delivery of core visual improvements:

1. Complete **Phase 1**: Setup (T001-T005)
2. Complete **Phase 2**: Foundational (T006-T014) ⚠️ CRITICAL
3. Complete **Phase 3**: User Story 1 - Enhanced Visual Identity (T015-T022)
4. Complete **Phase 4**: User Story 2 - Professional Chat Interface (T023-T030)
5. **STOP and VALIDATE**: Test both P1 stories independently
6. Deploy/demo MVP

**MVP delivers**: Custom typography, hero animations, week card hover effects, custom SVG chat icon

### Incremental Delivery (Priority Order)

1. **Setup + Foundational** → Foundation ready
2. **Add P1 Stories** (US1, US2) → Test independently → Deploy/Demo (MVP!)
3. **Add P2 Stories** (US3, US4) → Test independently → Deploy/Demo
4. **Add P3 Stories** (US5) → Test independently → Deploy/Demo
5. **Add P4 Stories** (US6-US9) → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (Phase 1-2)
2. Once Foundational is done, split by priority:
   - **Sprint 1**: P1 stories
     - Developer A: US1 (Enhanced Visual Identity)
     - Developer B: US2 (Professional Chat Interface)
   - **Sprint 2**: P2 stories
     - Developer A: US3 (Module Theming)
     - Developer B: US4 (Navigation & Progress)
   - **Sprint 3**: P3 story
     - Developer A: US5 (Loading States)
   - **Sprint 4**: P4 stories
     - Developer A: US6, US7 (3D Robot, Diagrams)
     - Developer B: US8, US9 (Quizzes, Effects)

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 95 |
| **Setup** | 5 |
| **Foundational** | 9 |
| **US1 - Visual Identity (P1)** | 8 |
| **US2 - Chat Interface (P1)** | 8 |
| **US3 - Module Theming (P2)** | 6 |
| **US4 - Navigation (P2)** | 8 |
| **US5 - Loading States (P3)** | 9 |
| **US6 - 3D Robot (P4)** | 8 |
| **US7 - Diagrams (P4)** | 8 |
| **US8 - Quizzes (P4)** | 9 |
| **US9 - Advanced Effects (P4)** | 9 |
| **Polish** | 8 |
| **Parallelizable** | ~60 tasks marked [P] |
| **MVP Scope** | 30 tasks (Setup + Foundational + US1 + US2) |

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [US#] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All accessibility features must respect prefers-reduced-motion
- All color combinations must meet WCAG AA contrast requirements
- 3D features must have fallbacks for no WebGL support
