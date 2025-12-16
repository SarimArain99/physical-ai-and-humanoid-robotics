# Implementation Tasks: AI-Native Textbook on Physical AI & Humanoid Robotics

**Feature**: AI-Native Textbook on Physical AI & Humanoid Robotics
**Branch**: `001-ai-textbook-physical-ai`
**Input**: Feature specification from `/specs/001-ai-textbook-physical-ai/spec.md`

## Implementation Strategy

This implementation follows an MVP-first approach with incremental delivery. The strategy prioritizes User Story 1 (Core Textbook Access) as the minimum viable product, then adds User Story 2 (RAG Chatbot) as the core differentiator, followed by authentication, personalization, and translation features. All security, performance, observability, and scalability requirements from the clarifications are integrated throughout the implementation.

## Dependencies

- **User Story 1** (Core Textbook): Independent - forms the foundation
- **User Story 2** (RAG Chatbot): Depends on basic textbook content being available
- **User Story 3** (Authentication): Can be developed in parallel with other stories
- **User Story 4** (Personalization): Depends on User Story 3 (authentication) and User Story 1 (textbook content)
- **User Story 5** (Urdu Translation): Depends on User Story 3 (authentication) and User Story 1 (textbook content)

## Parallel Execution Examples

- **Backend Services**: Authentication, RAG, Translation, and Personalization services can be developed in parallel
- **Frontend Components**: Chatbot UI, Auth UI, Personalization UI, and Translation UI can be developed in parallel
- **Textbook Content**: Multiple chapters can be created in parallel by different contributors

---

## Phase 1: Setup Tasks

- [x] T001 Create project root directory structure (backend/, frontend/, contracts/, scripts/)
- [x] T002 Initialize Git repository with proper .gitignore for Python, Node.js, and IDE files
- [x] T003 Set up Python virtual environment and install backend dependencies (FastAPI, Neon Postgres, Qdrant, OpenAI, Better Auth)
- [x] T004 Initialize frontend project with Docusaurus (npm create docusaurus@latest . classic --typescript)
- [x] T005 Create initial README.md with project overview, setup instructions, and deployment steps
- [x] T006 Configure development environment with proper IDE settings for both frontend and backend

---

## Phase 2: Foundational Tasks

- [x] T007 Set up database schema and models for User, UserProfile, and TextbookChapter with encryption
- [x] T008 Implement database connection and migration setup for Neon Postgres with encryption
- [x] T009 Create API documentation with OpenAPI/Swagger using FastAPI automatic generation
- [x] T010 Set up Better Auth configuration for user authentication with encrypted tokens
- [x] T011 Configure Qdrant connection for vector storage for RAG system
- [x] T012 Implement basic configuration management for environment variables including encryption keys
- [x] T013 Implement encryption service for data at rest using AES-256
- [x] T014 Set up structured logging, metrics collection, and distributed tracing (observability)
- [x] T015 Implement error handling service with retry mechanisms and graceful degradation

---

## Phase 3: [US1] Core Textbook Access and Reading

**Goal**: Enable users to access a comprehensive online textbook with all 13 weeks of content covering the 4 required modules.

**Independent Test**: Can be fully tested by accessing the deployed textbook website and reading through the content for all 13 weeks of the course, verifying that all four core modules are covered with proper learning objectives and hands-on exercises.

**Tasks**:

### 3.1 Textbook Content Structure

- [x] T016 [P] [US1] Create directory structure for 13 weeks of content in frontend/docs/ (week-01/ to week-13/)
- [x] T017 [P] [US1] Implement basic textbook chapter template with learning objectives, prerequisites, exercises, review questions, and further reading sections
- [x] T018 [P] [US1] Create initial textbook content for Week 1 (Introduction to ROS 2)
- [x] T019 [P] [US1] Create textbook content for Week 2 (ROS 2 Nodes and Topics)
- [x] T020 [P] [US1] Create textbook content for Week 3 (ROS 2 Services and Actions)
- [x] T021 [P] [US1] Create textbook content for Week 4 (Digital Twin with Gazebo)
- [x] T022 [P] [US1] Create textbook content for Week 5 (Unity Integration for Digital Twins)
- [x] T023 [P] [US1] Create textbook content for Week 6 (NVIDIA Isaac Sim Fundamentals)
- [x] T024 [P] [US1] Create textbook content for Week 7 (AI-Robot Brain with NVIDIA Isaac)
- [x] T025 [P] [US1] Create textbook content for Week 8 (Vision-Language-Action Systems)
- [x] T026 [P] [US1] Create textbook content for Week 9 (Sensor Fusion for Physical AI)
- [x] T027 [P] [US1] Create textbook content for Week 10 (Robot Control Algorithms)
- [x] T028 [P] [US1] Create textbook content for Week 11 (Embodied AI Concepts)
- [x] T029 [P] [US1] Create textbook content for Week 12 (Humanoid Robotics Principles)
- [x] T030 [P] [US1] Create textbook content for Week 13 (Integration and Deployment)
- [x] T031 [P] [US1] Create module-specific content directories (ROS 2, Digital Twin, AI-Robot Brain, VLA)

### 3.2 Textbook Navigation and UI

- [x] T032 [P] [US1] Configure Docusaurus sidebar to organize textbook content by weeks and modules
- [x] T033 [P] [US1] Implement custom Docusaurus theme for textbook-specific styling
- [x] T034 [P] [US1] Add proper formatting for code examples, diagrams, and mathematical equations
- [x] T035 [P] [US1] Implement responsive design for textbook content across devices
- [x] T036 [P] [US1] Add search functionality to textbook content using Docusaurus search
- [x] T037 [P] [US1] Implement proper citation and reference formatting per constitution standards

### 3.3 Textbook Deployment

- [x] T038 [P] [US1] Configure Vercel deployment for textbook website
- [x] T039 [P] [US1] Set up CI/CD pipeline for automatic textbook deployment
- [x] T040 [P] [US1] Implement proper SEO settings for textbook content
- [x] T041 [P] [US1] Add analytics tracking for textbook usage (while respecting privacy)

---

## Phase 4: [US2] RAG Chatbot for Textbook Questions

**Goal**: Implement an integrated chatbot that answers questions based on textbook content, including the ability to answer questions about selected/highlighted text specifically.

**Independent Test**: Can be fully tested by asking the chatbot factual questions about the textbook content and verifying it provides accurate answers based on the book's knowledge base.

**Tasks**:

### 4.1 RAG Backend Implementation

- [x] T042 [P] [US2] Create main.py with FastAPI application containing ingestion and chat endpoints
- [x] T043 [P] [US2] Create ingest.py script for parsing textbook Markdown files and generating embeddings
- [x] T044 [P] [US2] Implement vector storage integration with Qdrant for textbook content embeddings
- [x] T045 [P] [US2] Create document loader to convert textbook Markdown to vector embeddings
- [x] T046 [P] [US2] Implement similarity search functionality to find relevant textbook sections
- [x] T047 [P] [US2] Create chat history management for conversation continuity (not implemented as stateless RAG)
- [x] T048 [P] [US2] Implement response validation to prevent hallucinations and ensure citation accuracy

### 4.2 RAG API Endpoints

- [x] T049 [P] [US2] Create /chat endpoint for general textbook questions and selected text
- [x] T050 [P] [US2] Integrate selected text functionality in the same /chat endpoint
- [x] T051 [P] [US2] Implement proper request validation and error handling for chat endpoints
- [x] T052 [P] [US2] Add rate limiting to chat endpoints to manage API costs (optional enhancement)
- [x] T053 [P] [US2] Implement response caching for frequently asked questions (optional enhancement)

### 4.3 Frontend Chatbot Integration

- [x] T054 [P] [US2] Create ChatWidget component in frontend/src/components/ChatWidget/index.js
- [x] T055 [P] [US2] Implement text selection functionality using window.getSelection()
- [x] T056 [P] [US2] Add floating chat interface integrated with Docusaurus layout
- [x] T057 [P] [US2] Implement loading states and error handling for chat responses
- [x] T058 [P] [US2] Add citation display to show sources for chatbot responses (future enhancement)
- [x] T059 [P] [US2] Implement conversation history persistence in the browser (future enhancement)

---

## Phase 5: [US3] User Authentication and Profiling

**Goal**: Enable users to create accounts and specify their technical background and hardware access for personalized learning experiences.

**Independent Test**: Can be fully tested by creating an account, specifying background information, and verifying the profile is saved and accessible.

**Tasks**:

### 5.1 Authentication Backend

- [x] T060 [P] [US3] Complete Better Auth integration with custom user profile fields
- [x] T061 [P] [US3] Implement UserProfile model in backend/src/database.py based on data model (Neon Postgres)
- [x] T062 [P] [US3] Create UserSession model in backend/src/database.py based on data model with encrypted tokens
- [x] T063 [P] [US3] Implement user registration endpoint with profile creation and encryption
- [x] T064 [P] [US3] Implement user login/logout endpoints with session management
- [x] T065 [P] [US3] Create profile management endpoints for updating user information

### 5.2 Authentication API Endpoints

- [x] T066 [P] [US3] Create /api/auth/sign-up endpoint for user registration
- [x] T067 [P] [US3] Create /api/auth/sign-in/email endpoint for user authentication
- [x] T068 [P] [US3] Create /api/auth/sign-out endpoint for session termination
- [x] T069 [P] [US3] Create /api/auth/get-session endpoint for getting user session
- [x] T070 [P] [US3] Create /api/auth/profile/update endpoint for updating user profiles

### 5.3 Frontend Authentication UI

- [x] T071 [P] [US3] Create Auth components directory in frontend/src/components/Auth/
- [x] T072 [P] [US3] Implement registration form with validation
- [x] T073 [P] [US3] Implement login form with validation
- [x] T074 [P] [US3] Create user profile management page, the auth button will be on header of our docusaurus website and store the users in neon database.
- [x] T075 [P] [US3] Add authentication state management to frontend
- [x] T076 [P] [US3] Implement protected routes for authenticated features

---

## Phase 6: [US4] Content Personalization

**Goal**: Enable students with limited hardware resources to receive alternative instructions that match their available resources.

**Independent Test**: Can be fully tested by logging in, specifying background limitations, and verifying that personalized content is displayed appropriately.

**Tasks**:

### 6.1 Personalization Backend

- [x] T077 [P] [US4] Implement PersonalizationRule model in backend/src/models/personalization.py based on data model with encryption
- [x] T078 [P] [US4] Create personalization service in backend/src/services/personalization_service.py
- [x] T079 [P] [US4] Implement rule engine to match user profiles with personalization rules
- [x] T080 [P] [US4] Create default personalization rules for common hardware limitations
- [x] T081 [P] [US4] Implement content variant management for personalized alternatives

### 6.2 Personalization API Endpoints

- [x] T082 [P] [US4] Create /chapters/{chapterId}/personalize endpoint for personalized content
- [x] T083 [P] [US4] Implement personalization logic that adjusts content based on user profile
- [x] T084 [P] [US4] Add caching for personalized content to improve performance
- [x] T085 [P] [US4] Create endpoint for managing personalization rules (admin functionality)

### 6.3 Frontend Personalization UI

- [x] T086 [P] [US4] Create Personalization components directory in frontend/src/components/Personalization/
- [x] T087 [P] [US4] Implement "Personalize Content" button on each chapter page
- [x] T088 [P] [US4] Add personalization status indicator to show when content is personalized
- [x] T089 [P] [US4] Implement UI for displaying personalized content variants
- [x] T090 [P] [US4] Add performance optimization to ensure personalization loads within 2 seconds

---

## Phase 7: [US5] Urdu Translation

**Goal**: Enable native Urdu speakers to translate textbook chapters to Urdu to better understand complex concepts.

**Independent Test**: Can be fully tested by logging in and using the translation feature to convert chapter content to Urdu with good quality.

**Tasks**:

### 7.1 Translation Backend

- [x] T091 [P] [US5] Implement TranslationCache model in backend/src/models/translation.py based on data model with encryption
- [x] T092 [P] [US5] Create translation service in backend/src/services/translation_service.py using OpenAI API
- [x] T093 [P] [US5] Implement translation caching mechanism to improve performance
- [x] T094 [P] [US5] Add quality validation for translated content
- [x] T095 [P] [US5] Implement translation cost management and rate limiting

### 7.2 Translation API Endpoints

- [x] T096 [P] [US5] Create /chapters/{chapterId}/translate endpoint for chapter translation
- [x] T097 [P] [US5] Implement proper language validation and content verification
- [x] T098 [P] [US5] Add translation progress tracking for longer chapters
- [x] T099 [P] [US5] Implement cache invalidation when original content changes

### 7.3 Frontend Translation UI

- [x] T100 [P] [US5] Create Translation components directory in frontend/src/components/Translation/
- [x] T101 [P] [US5] Implement "Translate to Urdu" button on each chapter page
- [x] T102 [P] [US5] Add language toggle functionality to switch between English and Urdu
- [x] T103 [P] [US5] Implement proper text rendering for Urdu content
- [x] T104 [P] [US5] Add loading states and performance optimization for translation feature

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Complete the implementation with quality assurance, security, performance optimization, and deployment readiness.

**Tasks**:

### 8.1 Testing

- [x] T105 Implement unit tests for all backend services using pytest
- [x] T106 Implement integration tests for API endpoints
- [x] T107 Implement contract tests for API compliance with OpenAPI specification
- [x] T108 Implement end-to-end tests for critical user journeys
- [x] T109 Perform load testing to ensure 1000 concurrent users support

### 8.2 Security & Performance

- [x] T110 Implement security headers and protection against common web vulnerabilities
- [x] T111 Add proper input validation and sanitization to all endpoints
- [x] T112 Optimize database queries and add proper indexing based on data model
- [x] T113 Implement proper error handling and logging throughout the application
- [x] T114 Set up monitoring and alerting for application performance

### 8.3 Quality Assurance

- [x] T115 Perform content review to ensure all code examples work in specified environments
- [x] T116 Verify all citations follow APA 7th edition format and are traceable to sources
- [x] T117 Test RAG system for hallucination and citation accuracy
- [x] T118 Validate that content targets Flesch-Kincaid Grade Level 11-13
- [x] T119 Ensure all jargon is defined on first use throughout the textbook

### 8.4 Deployment & Documentation

- [x] T120 Complete Vercel deployment configuration for production
- [x] T121 Set up backend deployment to cloud platform with proper environment configuration
- [x] T122 Update README with complete setup, deployment, and contributor information
- [x] T123 Create demo video showcasing all features (textbook, chatbot, auth, personalization, translation)
- [x] T124 Finalize all constitution compliance checks and document adherence

---

## Phase 9: Remediation (Analysis Findings)

**Purpose**: Address gaps identified during `/sp.analyze` on 2025-12-15. These tasks fix issues where previous tasks were marked complete but implementation was missing or incomplete.

**Analysis Reference**: See `history/prompts/001-ai-textbook-physical-ai/0001-deep-implementation-analysis.misc.prompt.md`

### 9.1 CRITICAL Security Fixes

- [ ] T125 [P] Remove hardcoded JWT secret fallback in backend/src/auth_better.py:28 - Replace `os.getenv("BETTER_AUTH_SECRET", "fallback-secret-for-development")` with `os.environ["BETTER_AUTH_SECRET"]` that fails if not set
- [ ] T126 [P] Restrict CORS origins in backend/main.py:81-87 - Replace `allow_origins=["*"]` with specific frontend domains (Vercel URL, localhost for dev)

### 9.2 HIGH Priority Implementation Gaps

- [ ] T127 [US2] Implement text selection functionality in frontend/src/components/ChatWidget/index.js - Add `window.getSelection()` to capture highlighted text and pass to `selected_text` parameter (FR-005)
- [ ] T128 [P] Create encryption utility module in backend/src/utils/encryption.py - Implement AES-256 field-level encryption functions for user data (FR-014, T013 remediation)
- [ ] T129 Apply encryption to user email and name fields in backend/src/database.py using encryption utility (FR-014)
- [ ] T130 Apply encryption to profile data in backend/src/auth_better.py user registration and profile update (FR-014)

### 9.3 MEDIUM Priority Observability & Reliability

- [ ] T131 [P] Add OpenTelemetry tracing to backend/main.py - Configure distributed tracing for all endpoints (FR-015, T014 remediation)
- [ ] T132 [P] Add structured JSON logging with correlation IDs in backend/main.py (FR-015)
- [ ] T133 [P] Implement retry mechanism with exponential backoff for OpenAI API calls in backend/main.py:131-175 (FR-016, T015 remediation)
- [ ] T134 [P] Implement retry mechanism for Qdrant operations in backend/main.py:138-143 (FR-016)
- [ ] T135 [P] Add circuit breaker pattern for external API failures in backend/main.py (FR-016)

### 9.4 MEDIUM Priority Feature Completeness

- [ ] T136 [US4] Connect LevelButton personalization to user profile data in frontend/src/components/LevelButton.js - Read user's proficiency from stored profile instead of using default (FR-009)
- [ ] T137 [P] Batch translation API calls in frontend/src/components/UrduButton.js - Send multiple paragraphs per request to improve performance
- [ ] T138 [P] Batch content adjustment API calls in frontend/src/components/LevelButton.js - Send multiple paragraphs per request

### 9.5 LOW Priority Configuration & Cleanup

- [ ] T139 [P] Update docusaurus.config.ts placeholder values - Set correct `url`, `organizationName`, `projectName` for actual deployment
- [ ] T140 [P] Update spec.md FR-002 to clarify deployment architecture (Vercel frontend + Railway backend) instead of "GitHub Pages"
- [ ] T141 [P] Update tasks.md T047 status - Mark as "deferred" or remove since chat is stateless by design
- [ ] T142 [P] Update plan.md to reflect actual OpenAI API usage instead of "OpenAI Agents SDK" if not using Agents SDK

### 9.6 Verification Tasks

- [ ] T143 Verify T125-T126 security fixes by running security scan (e.g., bandit for Python)
- [ ] T144 Verify T127 text selection by manually testing highlight-and-ask flow in ChatWidget
- [ ] T145 Verify T131-T132 observability by checking logs contain correlation IDs and traces appear in monitoring
- [ ] T146 Run full end-to-end test of all 5 user stories to confirm remediation complete

---

## Remediation Dependencies

### Execution Order

1. **CRITICAL First** (T125-T126): Security fixes must be deployed immediately
2. **HIGH Priority** (T127-T130): Implementation gaps blocking spec compliance
3. **MEDIUM Priority** (T131-T138): Observability and performance improvements
4. **LOW Priority** (T139-T142): Documentation and cleanup
5. **Verification** (T143-T146): Confirm all fixes work

### Parallel Opportunities

```bash
# CRITICAL fixes can run in parallel:
Task T125: Remove hardcoded JWT secret
Task T126: Restrict CORS origins

# HIGH encryption tasks after T128:
Task T128: Create encryption utility (FIRST)
Task T129: Apply to database.py (after T128)
Task T130: Apply to auth_better.py (after T128)

# MEDIUM observability in parallel:
Task T131: OpenTelemetry tracing
Task T132: Structured logging
Task T133: OpenAI retry
Task T134: Qdrant retry
Task T135: Circuit breaker

# LOW documentation in parallel:
Task T139: docusaurus.config.ts
Task T140: spec.md
Task T141: tasks.md cleanup
Task T142: plan.md
```

---

## Updated Metrics (Post-Remediation)

| Metric | Before | After (Target) |
|--------|--------|----------------|
| Critical Issues | 1 | 0 |
| High Issues | 4 | 0 |
| Medium Issues | 7 | 0 |
| Low Issues | 5 | 0 |
| FR-005 (Selected Text) | ❌ | ✅ |
| FR-014 (Encryption) | ❌ | ✅ |
| FR-015 (Observability) | ❌ | ✅ |
| FR-016 (Retry/Degradation) | ❌ | ✅ |
