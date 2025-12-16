---
id: 0011
title: Chat history auth bug investigation
stage: tasks
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 001-ai-textbook-physical-ai
branch: main
user: sarimarain99
command: /sp.tasks I have checked the chat history but it is not being updated even I have signed-in and still showing (Please sign in to view chat history) but not showing my history.
labels: ["phase-10", "bug-fix", "authentication", "chat-history", "frontend"]
links:
  spec: specs/001-ai-textbook-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook-physical-ai/tasks-bugfix-chat-history-auth.md
  - frontend/src/components/ChatWidget/ChatHistory.js
  - frontend/src/components/Auth/AuthProvider.js
tests:
  - Manual testing required - Sign in and check if history loads
  - Test race conditions between AuthProvider and ChatHistory
---

## Prompt

I have checked the chat history but it is not being updated even I have signed-in and still showing (Please sign in to view chat history) but not showing my history.

## Response snapshot

Investigated the bug and identified root cause:

**Problem**: ChatHistory component checks `localStorage.getItem('auth_token')` directly instead of using the `user` context from AuthProvider.

**Root Cause Analysis**:
- Location: `frontend/src/components/ChatWidget/ChatHistory.js` lines 46-51
- Issue: Component bypasses AuthProvider and checks localStorage directly
- This creates a race condition where:
  1. Token exists in localStorage
  2. But AuthProvider hasn't validated it yet
  3. Or token is expired but still in localStorage
  4. ChatHistory shows error even though user is signed in

**AuthProvider Flow** (correct):
1. Checks localStorage for token
2. Validates token with backend (`/api/auth/get-session`)
3. Sets `user` object only if token is valid
4. Provides `user` and `isAuthenticated` to components

**ChatHistory Flow** (buggy):
1. Directly checks localStorage for token
2. Skips AuthProvider validation
3. May show error even when user is authenticated

**Generated Tasks**:
Created comprehensive bug fix task list with 19 tasks (T186-T204):
- Phase 1: Investigation & root cause confirmation (T186-T189)
- Phase 2: Fix implementation (T190-T195)
- Phase 3: Verification & testing (T196-T200)
- Phase 4: Related fixes (T201-T204)

**Solution**:
1. Update ChatHistory to accept `user` prop from parent
2. Remove direct localStorage check
3. Use `if (!user)` instead of `if (!token)`
4. Pass `user` prop from ChatWidget to ChatHistory
5. Add auth loading state handling

## Outcome

- ✅ Impact: Identified critical authentication bug affecting all users. Root cause confirmed. 19 tasks generated to fix bug, verify all auth flows, and ensure consistency across components.
- 🧪 Tests: Manual testing plan included - Test sign in, sign out, reload, race conditions, invalid tokens
- 📁 Files: 1 new task file created, 2 files identified for fixes (ChatHistory.js, ChatWidget/index.js)
- 🔁 Next prompts: Implement bug fix tasks T186-T204, starting with investigation to confirm, then fix implementation
- 🧠 Reflection: The bug stems from not properly using React context patterns. ChatHistory should rely on AuthProvider state rather than directly accessing localStorage. This is a common anti-pattern in React apps - bypassing context providers leads to state synchronization issues.

## Evaluation notes (flywheel)

- Failure modes observed: Race condition between AuthProvider initialization and ChatHistory mount. Token validation happens async but ChatHistory checks sync.
- Graders run and results (PASS/FAIL): Bug confirmed through code analysis. Manual testing pending for verification.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Start with T190 - Add `user` prop to ChatHistory and test if error goes away
