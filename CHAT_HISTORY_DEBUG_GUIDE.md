# Chat History Authentication Debug Guide

**Issue**: Chat history showing authentication errors despite being signed in
**Status**: Enhanced diagnostics added
**Date**: 2025-12-16

## Current Error Messages

1. **"Please sign in to view chat history"** - User prop is null
2. **"Authentication error. Please try logging out and back in."** - User exists but no token in localStorage

## Diagnostic Steps

If you see authentication errors in chat history, follow these steps:

### Step 1: Check Browser Console

Open your browser's Developer Tools (F12) and check the Console tab for error messages:

- Look for `ChatHistory: No token despite user being authenticated`
- This will show you the user object and whether the token exists

### Step 2: Check localStorage

In the Console tab, run:
```javascript
localStorage.getItem('auth_token')
```

**Expected**: Should return a long string (your JWT token)
**If null**: Token is missing - this is the problem!

### Step 3: Check AuthProvider State

In the Console, run:
```javascript
console.log(document.querySelector('[data-test-id="chat-widget"]'))
```

Or check React DevTools to see the AuthProvider state.

### Step 4: Check Network Tab

1. Open Network tab in DevTools
2. Click the History button in chat
3. Look for the request to `/api/chat/sessions`
4. Check if it has an `Authorization: Bearer <token>` header

## Common Causes & Solutions

### Cause 1: Token Cleared But User State Persists

**Symptoms**: User object exists, but localStorage.getItem('auth_token') returns null

**Solution**:
1. Log out completely
2. Clear browser cache and cookies
3. Log back in
4. Try accessing chat history

**Commands**:
```javascript
// Clear localStorage
localStorage.clear();

// Reload page
location.reload();
```

### Cause 2: Better Auth Session Mismatch

**Symptoms**: Token exists but API returns 401/403

**Solution**: Token might be expired or invalid

1. Check token expiration:
```javascript
const token = localStorage.getItem('auth_token');
const parts = token.split('.');
const payload = JSON.parse(atob(parts[1]));
console.log('Token expires:', new Date(payload.exp * 1000));
console.log('Current time:', new Date());
```

2. If expired, log out and log back in

### Cause 3: Backend /api/auth/get-session Failing

**Symptoms**: AuthProvider sets user=null even with valid token

**Check Backend**:
1. Open Network tab
2. Find request to `/api/auth/get-session`
3. Check response status and body
4. If 401/403, token is invalid
5. If 500, backend error

**Solution**: Check backend logs, verify Better Auth configuration

### Cause 4: CORS or Network Issues

**Symptoms**: Requests fail silently

**Check**:
1. Network tab shows failed requests (red)
2. Console shows CORS errors
3. Backend URL might be wrong

**Verify Backend URL**:
```javascript
console.log('Backend URL in code:');
// Should be: https://physical-ai-and-humanoid-robotics-production.up.railway.app
```

## Code Changes Made

### File: `frontend/src/components/ChatWidget/ChatHistory.js`

**Changes**:
1. Added SSR safety check for localStorage
2. Enhanced error logging with user and token details
3. Clear error messages for different scenarios
4. Better error message: "Authentication error. Please try logging out and back in."

**Key Logic**:
```javascript
// 1. Check if auth is loading
if (authLoading) return;

// 2. Check if user exists (from AuthProvider)
if (!user) {
  setError('Please sign in...');
  return;
}

// 3. Get token from localStorage
const token = localStorage.getItem('auth_token');

// 4. If no token but user exists = AUTH BUG
if (!token) {
  console.error('No token despite user authenticated');
  setError('Authentication error. Please try logging out and back in.');
  return;
}

// 5. Make API call with token
fetch(url, { headers: { Authorization: `Bearer ${token}` } })
```

## Expected Behavior

**When signed in**:
1. AuthProvider validates token with `/api/auth/get-session`
2. Sets `user` state
3. Token remains in localStorage
4. ChatHistory reads user and token
5. Makes API call to `/api/chat/sessions`
6. Sessions load successfully

**When NOT signed in**:
1. No token in localStorage
2. AuthProvider sets user=null
3. ChatHistory shows "Please sign in to view chat history"
4. User clicks sign in button

## Manual Testing Checklist

- [ ] Sign in with valid credentials
- [ ] Check console - no errors
- [ ] Check localStorage - token exists
- [ ] Click History button
- [ ] Verify sessions load OR see error with diagnostic info
- [ ] Check console for detailed error if it fails
- [ ] Log out
- [ ] Verify "Please sign in" message appears
- [ ] Log back in
- [ ] Verify history works

## If Issue Persists

1. **Collect diagnostic info from console**:
   - User object details
   - Token existence
   - localStorage keys
   - Network request/response

2. **Check Backend**:
   - Is Better Auth configured correctly?
   - Does `/api/auth/get-session` endpoint work?
   - Are JWT tokens being generated correctly?
   - Check backend logs for errors

3. **Verify Frontend-Backend Connection**:
   - CORS configured?
   - Railway URL correct?
   - Network connectivity?

## Contact Points

- Frontend: ChatHistory.js, AuthProvider.js
- Backend: /api/auth/get-session, /api/chat/sessions
- Auth System: Better Auth integration

## Next Steps

Run the diagnostic steps above and share:
1. Console error messages (screenshot)
2. localStorage.getItem('auth_token') output
3. Network tab screenshot of /api/chat/sessions request
4. User object from console

This will help identify the exact root cause.
