---
id: 0015
title: Chat history token diagnostics and error handling
stage: green
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 001-ai-textbook-physical-ai
branch: main
user: sarimarain99
command: /sp.implement now chathistory showing this error(Session expired - please refresh the page) I refreshed page but still same.
labels: ["phase-10", "bug-fix", "authentication", "diagnostics", "error-handling", "chat-history"]
links:
  spec: specs/001-ai-textbook-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/ChatWidget/ChatHistory.js
  - CHAT_HISTORY_DEBUG_GUIDE.md
tests:
  - Manual testing with console diagnostics
  - Check localStorage for token
  - Verify AuthProvider state
---

## Prompt

/sp.implement now chathistory showing this error(Session expired - please refresh the page) I refreshed page but still same.

Context: User seeing persistent authentication error even after refresh. This indicates token is missing from localStorage despite user being authenticated.

## Response snapshot

The persistent error reveals a **fundamental authentication state mismatch**: User prop is set but token is not in localStorage. This shouldn't be possible if AuthProvider is working correctly.

**Root Cause Analysis**:

The error "Session expired - please refresh the page" appears when:
1. `user` prop exists (AuthProvider set it)
2. `localStorage.getItem('auth_token')` returns `null`

**This indicates one of three scenarios**:
1. **Token was cleared** but user state persisted in React
2. **AuthProvider bug**: Setting user without token in localStorage
3. **Better Auth issue**: Session validated but token not saved

**Enhanced Diagnostics Added**:

```javascript
// Added SSR safety check
const token = typeof window !== 'undefined' && typeof localStorage !== 'undefined'
  ? localStorage.getItem('auth_token')
  : null;

// Enhanced error logging
if (!token) {
  console.error('ChatHistory: No token despite user being authenticated', {
    user: user?.email || user?.name,
    userObject: user
  });
  setError('Authentication error. Please try logging out and back in.');
}
```

**Debug Guide Created**: `CHAT_HISTORY_DEBUG_GUIDE.md`

Comprehensive debugging document with:
- **Diagnostic Steps**: Check console, localStorage, network tab
- **Common Causes**: Token cleared, session mismatch, backend issues, CORS
- **Solutions**: Clear cache, log out/in, verify backend, check token expiration
- **Manual Testing Checklist**: Step-by-step verification
- **Code Explanation**: How authentication flow should work

**What User Should Do**:

1. **Open Browser DevTools** (F12)
2. **Check Console** for error message with details
3. **Run in Console**:
   ```javascript
   localStorage.getItem('auth_token')
   ```
   - If `null` → Token is missing (problem confirmed)
   - If shows token → Token exists (different issue)

4. **If token is missing**:
   ```javascript
   localStorage.clear();
   location.reload();
   ```
   Then log in again

5. **Check Network Tab**:
   - Look for `/api/auth/get-session` request
   - Check response status
   - Verify token is returned

**Possible Root Causes**:

1. **Better Auth Integration Issue**:
   - AuthProvider's `checkUser()` function calls `/api/auth/get-session`
   - If response is OK, sets user
   - BUT doesn't verify token is still in localStorage
   - Token could have been cleared by another process

2. **Race Condition**:
   - Token gets cleared after AuthProvider sets user
   - Or token never gets saved during login

3. **Backend Issue**:
   - `/api/auth/get-session` returns user data
   - But token in localStorage is invalid/expired
   - Backend doesn't reject invalid tokens properly

**Immediate Actions for User**:

Try these in order:
1. **Log out and log back in** (clears all state)
2. **Clear browser cache and cookies**
3. **Check console for diagnostic info**
4. **Verify backend is running and responding**

**Code Changes**:
- Enhanced error logging with user details
- SSR safety checks for localStorage
- Clearer error message suggesting logout/login
- Created comprehensive debug guide

## Outcome

- ✅ Impact: Added detailed diagnostics and error logging to identify root cause of persistent token issue. Created comprehensive debug guide for troubleshooting. User now has clear error message suggesting logout/login instead of refresh.
- 🧪 Tests: User needs to test - check console for diagnostic info, verify localStorage state, try logout/login
- 📁 Files: 2 files modified/created (ChatHistory.js enhanced logging, CHAT_HISTORY_DEBUG_GUIDE.md created)
- 🔁 Next prompts: User should run diagnostic steps and report findings. May need to investigate AuthProvider or Better Auth integration based on results.
- 🧠 Reflection: This is likely an AuthProvider integration issue, not a ChatHistory bug. AuthProvider is setting user without ensuring token exists in localStorage, or something is clearing the token. The real fix might need to be in AuthProvider or Better Auth configuration, not ChatHistory.

## Evaluation notes (flywheel)

- Failure modes observed: Persistent token missing despite user authenticated suggests AuthProvider/Better Auth integration issue
- Graders run and results (PASS/FAIL): Enhanced diagnostics in place. Root cause investigation needed - likely backend/auth system issue.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): User should check localStorage.getItem('auth_token') in console and share result, also check /api/auth/get-session response in Network tab
