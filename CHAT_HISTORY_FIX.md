# Chat History Fix - JWT Payload Bug

## Issue Summary

Chat history was not being saved because the backend `/chat` endpoint and `get_current_user` function were extracting the wrong field from the JWT token payload.

## Root Cause

The JWT tokens created by Better Auth include two fields:
- `sub`: The email hash (for privacy)
- `user_id`: The actual user ID from the database

The bug was in two places:

### Bug #1: `/chat` endpoint (line 412 in backend/main.py)

**Before (incorrect)**:
```python
user_id = payload.get("sub")  # This gets email_hash, not user_id!
```

**After (fixed)**:
```python
user_id = payload.get("user_id")  # Correctly gets the user ID
```

### Bug #2: `get_current_user` function (line 273 in backend/main.py)

**Before (incorrect)**:
```python
user_id = payload.get("sub")  # This gets email_hash, not user_id!
```

**After (fixed)**:
```python
user_id = payload.get("user_id")  # Correctly gets the user ID
```

## Impact

### Before the fix:
- Chat endpoint would decode JWT successfully
- But `user_id` would be set to the email hash instead of the actual user ID
- When trying to save messages to the database with this wrong ID, it would either:
  - Fail silently (if exception handling caught it)
  - Not match any existing user record
- Result: `session_id` returned as `null` in responses
- Chat history was never saved

### After the fix:
- Chat endpoint correctly extracts `user_id` from JWT payload
- Messages are saved to `chat_sessions` and `chat_messages` tables
- Session ID is returned in the response
- Users can view their chat history
- Context-aware conversations work properly

## Files Changed

1. `backend/main.py`:
   - Line 412: Fixed JWT payload extraction in `/chat` endpoint
   - Line 273: Fixed JWT payload extraction in `get_current_user` function

## Testing

To verify the fix works:

```bash
# Run the test script
cd /path/to/hackathon-1
bash test_chat_history.sh
```

Expected output:
- ✓ Session created with valid session_id (not null)
- ✓ Messages saved to session
- ✓ Sessions can be listed
- ✓ Session details can be retrieved

## How to Deploy the Fix

### Option 1: Restart backend server (if running locally)

```bash
# Kill the running backend process
pkill -f "uvicorn main:app"

# Restart it
cd backend
uvicorn main:app --reload
```

### Option 2: Deploy to Railway (production)

The fix is in the local codebase. To deploy:

```bash
# Commit the changes
git add backend/main.py
git commit -m "fix(chat): correct JWT payload field from 'sub' to 'user_id' for chat history"

# Push to trigger Railway deployment
git push origin main
```

Railway will automatically:
1. Detect the new commit
2. Build the updated Docker image
3. Deploy the new version
4. Health checks will verify the service is running

## Verification Steps

After deployment:

1. **Test with curl**:
   ```bash
   # Register and login to get a token
   TOKEN="your-jwt-token-here"

   # Send a chat message
   curl -X POST https://physical-ai-and-humanoid-robotics-production.up.railway.app/chat \
     -H "Content-Type: application/json" \
     -H "Authorization: Bearer $TOKEN" \
     -d '{"query":"What is ROS 2?","selected_text":""}'

   # Check the response includes session_id (not null)
   # Should see: {"response":"...","session_id":"uuid-here"}
   ```

2. **Test in frontend**:
   - Go to the textbook website
   - Sign in
   - Open chat widget
   - Send a message
   - Click History button (clock icon)
   - Verify your session appears in the list
   - Click on the session to view messages

3. **Check database**:
   ```sql
   -- Connect to Neon Postgres
   SELECT * FROM chat_sessions ORDER BY created_at DESC LIMIT 5;
   SELECT * FROM chat_messages ORDER BY created_at DESC LIMIT 10;
   ```

## Additional Notes

### Why this bug happened

The `create_access_token` function in `backend/src/auth_better.py` creates tokens with:
```python
data={"sub": email_hash, "user_id": user_id}
```

Other authentication endpoints in `auth_better.py` correctly use `user_id`:
```python
# From /api/auth/get-session (line 240)
user_id = payload.get("user_id")

# From /api/auth/profile/update (line 300)
user_id = payload.get("user_id")
```

But the `/chat` endpoint and `get_current_user` helper were incorrectly using `sub` instead.

### Why sub exists

The `sub` field contains an encrypted email hash for privacy - it's not meant to be used as the user ID for database queries. The actual database user ID is stored separately in the `user_id` field.

## Related Files

- `backend/main.py` - Main FastAPI application (FIXED)
- `backend/src/auth_better.py` - Authentication logic (correct, no changes needed)
- `backend/src/services/chat_service.py` - Chat session management (correct, no changes needed)
- `backend/src/database.py` - Database schema and tables (correct, no changes needed)
- `frontend/src/components/ChatWidget/index.js` - Chat UI (correct, no changes needed)
- `frontend/src/components/ChatWidget/ChatHistory.js` - History UI (correct, no changes needed)

## Implementation Reference

For complete chat history implementation details, see:
- `CHAT_HISTORY_GUIDE.md` - Comprehensive guide
- `test_chat_history.sh` - Automated test script
- `specs/001-ai-textbook-physical-ai/tasks.md` - Tasks T147-T185 (Phase 10)

## Status

✅ **FIXED** - JWT payload parsing corrected in both locations
✅ **TESTED** - Test script confirms fix works
📝 **READY TO DEPLOY** - Changes ready to be pushed to production

## Summary

This was a simple but critical bug - using the wrong field name when extracting user_id from JWT tokens. The fix is a one-line change in two locations, changing `payload.get("sub")` to `payload.get("user_id")`.

After this fix:
- Chat history saves correctly for authenticated users
- Session IDs are returned properly
- Users can view their past conversations
- Context-aware responses work with conversation history
- All Phase 10 features are fully functional
