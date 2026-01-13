---

description: "Task list for Hugging Face backend deployment migration"
---

# Tasks: Hugging Face Backend Deployment

**Input**: Design documents from `/specs/001-huggingface/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/backend-api.yaml

**Tests**: Tests are NOT explicitly requested in this feature specification. Deployment verification will be done through health endpoint checks and manual validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` at repository root
- All modifications are to existing backend files except new README.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing project structure and prepare for Hugging Face deployment

- [X] T001 Verify existing backend structure matches plan.md in backend/
- [X] T002 Verify Dockerfile uses Python 3.11 slim base image in backend/Dockerfile
- [X] T003 Verify PORT environment variable handling in backend/Dockerfile
- [X] T004 Verify .dockerignore excludes unnecessary files in backend/.dockerignore
- [X] T005 Verify all dependencies listed in backend/requirements.txt

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core configuration changes that MUST be complete before user stories can be verified

**⚠️ CRITICAL**: No deployment verification can begin until this phase is complete

- [X] T006 Modify config.py to make env_file optional in backend/config.py
- [X] T007 Add clear error messages for missing required configuration values in backend/config.py
- [X] T008 Remove Railway-specific URLs from default CORS origins in backend/main.py

**Checkpoint**: Foundation ready - Hugging Face deployment can now be tested

---

## Phase 3: User Story 1 - Deploy Backend to Hugging Face (Priority: P1) 🎯 MVP

**Goal**: Successfully deploy the FastAPI backend to Hugging Face Spaces with Docker SDK

**Independent Test**: Deploy to Hugging Face Space and verify `/health` endpoint returns `{"status": "healthy"}` within 2 seconds

### Implementation for User Story 1

- [X] T009 [P] [US1] Verify Dockerfile CMD uses ${PORT} variable correctly in backend/Dockerfile
- [X] T010 [P] [US1] Verify health endpoint returns proper status in backend/main.py
- [X] T011 [US1] Verify external service connection logging in startup lifecycle in backend/main.py
- [X] T012 [US1] Remove Railway-specific nixpacks.toml file from backend/nixpacks.toml
- [X] T013 [US1] Create Hugging Face specific README.md with YAML frontmatter in backend/README.md

**Checkpoint**: At this point, backend should deploy to HF and `/health` should return healthy status

---

## Phase 4: User Story 2 - Environment Configuration via Secrets (Priority: P2)

**Goal**: Configure all required service credentials through Hugging Face Secrets without `.env` file dependency

**Independent Test**: Start application without `.env` file present and verify all configuration loads from environment variables

### Implementation for User Story 2

- [X] T014 [P] [US2] Verify env_file optional logic handles missing .env gracefully in backend/config.py
- [X] T015 [P] [US2] Test configuration loads without .env file in backend/config.py
- [X] T016 [US2] Verify optional configuration values have sensible defaults in backend/config.py
- [X] T017 [US2] Document all environment variables in backend/README.md

**Checkpoint**: At this point, application should start without .env file using only HF Secrets

---

## Phase 5: User Story 3 - Frontend CORS Access (Priority: P2)

**Goal**: Frontend applications can make API requests to deployed backend without CORS errors

**Independent Test**: Make cross-origin request from frontend to deployed HF Space URL and verify success

### Implementation for User Story 3

- [X] T018 [US3] Verify CORS_ALLOWED_ORIGINS environment variable parsing in backend/main.py
- [X] T019 [US3] Verify credentials-based CORS is enabled in backend/main.py
- [X] T020 [US3] Add Hugging Face Space URL pattern to documentation in backend/README.md
- [X] T021 [US3] Document CORS configuration examples in backend/README.md

**Checkpoint**: At this point, frontend should successfully make CORS requests to deployed backend

---

## Phase 6: User Story 4 - Production Health Monitoring (Priority: P3)

**Goal**: Operators can monitor deployed service health and circuit breaker status

**Independent Test**: Call `/health/circuits` endpoint and verify it returns circuit breaker states

### Implementation for User Story 4

- [X] T022 [P] [US4] Verify /health endpoint exists and returns status in backend/main.py
- [X] T023 [P] [US4] Verify /health/circuits endpoint returns circuit states in backend/main.py
- [X] T024 [US4] Verify root endpoint / returns service status in backend/main.py
- [X] T025 [US4] Add health endpoint documentation in backend/README.md

**Checkpoint**: At this point, all health monitoring endpoints should be documented and functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final documentation, validation, and deployment readiness

- [X] T026 [P] Verify all environment variables documented with descriptions in backend/README.md
- [X] T027 [P] Add Hugging Face deployment instructions to backend/README.md
- [X] T028 [P] Add troubleshooting guide for common deployment issues to backend/README.md
- [X] T029 Verify Docker build completes in under 20 minutes (test locally if possible)
- [X] T030 Verify README.md includes Hugging Face Spaces YAML metadata in backend/README.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - verification tasks can run immediately
- **Foundational (Phase 2)**: Depends on Setup verification - BLOCKS deployment testing
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (Deployment) is P1 - primary MVP
  - User Story 2 (Config) and User Story 3 (CORS) are P2 - can proceed in parallel
  - User Story 4 (Monitoring) is P3 - can proceed independently
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - Independent deployment capability
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent configuration verification
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Independent CORS testing
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Independent monitoring verification

### Within Each User Story

- [P] marked tasks can run in parallel within same phase
- File modification tasks should be sequenced to avoid conflicts
- Documentation tasks can run in parallel with code changes

### Parallel Opportunities

- **Setup Phase**: T001, T002, T003, T004, T005 can all run in parallel
- **Foundational Phase**: T006, T007, T008 have some dependencies (config.py before main.py)
- **User Story 1**: T009, T010 can run in parallel
- **User Story 2**: T014, T015 can run in parallel
- **User Story 3**: Single sequential flow
- **User Story 4**: T022, T023, T024 can run in parallel
- **Polish Phase**: T026, T027, T028, T030 can run in parallel

---

## Parallel Example: User Story 1 (Deployment)

```bash
# Launch verification tasks in parallel:
Task: "T009 [P] [US1] Verify Dockerfile CMD uses ${PORT} variable correctly"
Task: "T010 [P] [US1] Verify health endpoint returns proper status"

# Then run sequential tasks:
Task: "T011 [US1] Verify external service connection logging"
Task: "T012 [US1] Remove Railway-specific nixpacks.toml"
Task: "T013 [US1] Create Hugging Face specific README.md"
```

---

## Parallel Example: Setup Phase

```bash
# Launch all verification tasks in parallel:
Task: "T001 Verify existing backend structure matches plan.md"
Task: "T002 Verify Dockerfile uses Python 3.11 slim base image"
Task: "T003 Verify PORT environment variable handling"
Task: "T004 Verify .dockerignore excludes unnecessary files"
Task: "T005 Verify all dependencies listed in requirements.txt"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (verification tasks)
2. Complete Phase 2: Foundational (config.py and main.py modifications)
3. Complete Phase 3: User Story 1 (deployment verification)
4. **STOP and VALIDATE**: Deploy to Hugging Face Space and test `/health` endpoint
5. Proceed to additional stories only if MVP requires them

### Incremental Delivery

1. Complete Setup + Foundational → HF deployment ready
2. Add User Story 1 → Deploy and test `/health` → MVP Complete!
3. Add User Story 2 → Test without .env file → Config validation complete
4. Add User Story 3 → Test CORS from frontend → Integration complete
5. Add User Story 4 → Verify monitoring endpoints → Production ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Deployment)
   - Developer B: User Story 2 (Config)
   - Developer C: User Story 3 (CORS)
3. Developer D: User Story 4 (Monitoring) - can start in parallel
4. All converge on Polish phase

---

## Files Modified/Created

| File | Type | Purpose |
|------|------|---------|
| `backend/config.py` | MODIFY | Make env_file optional, add validation |
| `backend/main.py` | MODIFY | Update CORS origins, verify health endpoints |
| `backend/README.md` | CREATE | HF deployment documentation |
| `backend/nixpacks.toml` | DELETE | Railway-specific, not needed for HF |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently verifiable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All health endpoints already exist - verification only, no new code
