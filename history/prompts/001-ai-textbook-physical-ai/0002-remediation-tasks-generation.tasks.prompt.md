---
id: 0002
title: Remediation Tasks Generation
stage: tasks
date: 2025-12-15
surface: agent
model: claude-opus-4-5-20251101
feature: 001-ai-textbook-physical-ai
branch: main
user: developer
command: /sp.tasks
labels: ["tasks", "remediation", "security-fix", "implementation-gaps"]
links:
  spec: specs/001-ai-textbook-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook-physical-ai/tasks.md (modified)
tests:
  - N/A (task generation only)
---

## Prompt

/sp.tasks fix all things that are needed to fix.

User requested task generation to fix all issues identified in the previous `/sp.analyze` run.

## Response snapshot

Added **Phase 9: Remediation** to tasks.md with 22 new tasks (T125-T146) organized by priority:

**Task Breakdown:**
- CRITICAL Security (2 tasks): T125-T126 - JWT secret, CORS
- HIGH Implementation (4 tasks): T127-T130 - Text selection, encryption
- MEDIUM Observability (5 tasks): T131-T135 - Tracing, logging, retries
- MEDIUM Features (3 tasks): T136-T138 - Personalization, batching
- LOW Documentation (4 tasks): T139-T142 - Config cleanup
- Verification (4 tasks): T143-T146 - Testing fixes

**Files Modified:**
- specs/001-ai-textbook-physical-ai/tasks.md - Added Phase 9 with 22 remediation tasks

**Execution Order:**
1. CRITICAL first (T125-T126) - Deploy immediately
2. HIGH priority (T127-T130) - Spec compliance
3. MEDIUM priority (T131-T138) - Quality improvements
4. LOW priority (T139-T142) - Documentation
5. Verification (T143-T146) - Confirm fixes

## Outcome

- ✅ Impact: 22 new remediation tasks added to address all 17 findings from analysis
- 🧪 Tests: N/A (task generation)
- 📁 Files: 1 file modified (tasks.md)
- 🔁 Next prompts: /sp.implement to execute remediation tasks starting with T125
- 🧠 Reflection: Remediation tasks are concrete and actionable with exact file paths and line numbers

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard /sp.tasks with remediation context
- Next experiment (smallest change to try): Run /sp.implement to execute T125-T126 (CRITICAL) first
