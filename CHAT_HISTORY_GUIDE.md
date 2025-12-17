# Chat History Implementation Guide

## Overview

The chat history feature has been fully implemented for authenticated users. This allows users to:
- View all past conversations
- Continue previous chats
- Create new chat sessions
- Delete individual sessions
- Export chat history
- Clear all history

## Architecture

### Backend (FastAPI)

#### Database Tables

Two tables store chat history in Neon Postgres:

1. **chat_sessions** - Stores conversation sessions
   ```sql
   CREATE TABLE chat_sessions (
       id VARCHAR(36) PRIMARY KEY,
       user_id VARCHAR(36) NOT NULL,
       title VARCHAR(255) NOT NULL DEFAULT 'New Chat',
       is_active BOOLEAN DEFAULT TRUE,
       created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
       updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
   );
   ```

2. **chat_messages** - Stores individual messages
   ```sql
   CREATE TABLE chat_messages (
       id VARCHAR(36) PRIMARY KEY,
       session_id VARCHAR(36) NOT NULL,
       role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
       content TEXT NOT NULL,
       selected_text TEXT,
       sources TEXT,
       created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
       FOREIGN KEY (session_id) REFERENCES chat_sessions(id) ON DELETE CASCADE
   );
   ```

#### API Endpoints

| Endpoint | Method | Description | Auth Required |
|----------|--------|-------------|---------------|
| `/api/chat/sessions` | GET | List user's chat sessions | ✅ Yes |
| `/api/chat/sessions` | POST | Create new chat session | ✅ Yes |
| `/api/chat/sessions/{session_id}` | GET | Get session with messages | ✅ Yes |
| `/api/chat/sessions/{session_id}` | DELETE | Delete a session | ✅ Yes |
| `/api/chat/sessions/{session_id}/export` | GET | Export session (text/json) | ✅ Yes |
| `/api/chat/sessions/{session_id}/title` | PATCH | Update session title | ✅ Yes |
| `/api/chat/history` | DELETE | Clear all user's history | ✅ Yes |
| `/chat` | POST | Send message (auto-saves to session if authenticated) | ❌ No (but saves if authenticated) |

#### Chat Service (`backend/src/services/chat_service.py`)

The ChatService class handles all database operations:
- `create_session(user_id, title)` - Creates new session
- `get_user_sessions(user_id, limit, offset)` - Lists sessions with pagination
- `get_session_messages(session_id, user_id)` - Gets session with messages
- `add_message(session_id, role, content, selected_text)` - Adds message to session
- `delete_session(session_id, user_id)` - Deletes session (with ownership check)
- `clear_all_history(user_id)` - Deletes all user's sessions
- `export_session(session_id, user_id, format)` - Exports as text or JSON
- `get_active_session(user_id)` - Gets current active session
- `update_session_title(session_id, user_id, title)` - Updates session title

### Frontend (React + Docusaurus)

#### Components

1. **ChatWidget (`frontend/src/components/ChatWidget/index.js`)**
   - Main chat interface
   - Auto-loads last active session on open
   - Saves messages to session when authenticated
   - Shows history button for authenticated users

2. **ChatHistory (`frontend/src/components/ChatWidget/ChatHistory.js`)**
   - Lists all user's chat sessions
   - Displays session title, message count, last updated
   - Pagination support (10 sessions per page)
   - Actions: Select session, delete, export, clear all

3. **ChatSession (`frontend/src/components/ChatWidget/ChatSession.js`)**
   - Displays messages for a specific session
   - View-only mode to review past conversations

#### Authentication Flow

1. User logs in via `/login` page
2. Backend returns JWT access token
3. Frontend stores token in `localStorage` as `auth_token`
4. AuthProvider validates token on page load
5. ChatWidget checks `user` from AuthProvider
6. If authenticated, shows history button and saves messages

## How It Works

### Message Flow (Authenticated User)

```
1. User types message and clicks send
   ↓
2. ChatWidget sends POST to /chat with:
   - query: user message
   - selected_text: highlighted text (if any)
   - session_id: current session ID (if continuing)
   ↓
3. Backend checks if user is authenticated (JWT token)
   ↓
4. If authenticated:
   - Gets or creates chat session
   - Saves user message to chat_messages table
   - Generates RAG response
   - Saves assistant response to chat_messages table
   - Returns response + session_id
   ↓
5. Frontend receives response:
   - Displays assistant message
   - Stores session_id for next message
   - Updates local state
```

### Viewing History

```
1. User clicks History button (clock icon)
   ↓
2. ChatHistory component loads
   ↓
3. Fetches GET /api/chat/sessions with auth token
   ↓
4. Backend returns:
   {
     sessions: [
       {
         id: "uuid",
         title: "First question...",
         message_count: 5,
         is_active: true,
         created_at: "2025-12-17T10:00:00Z",
         updated_at: "2025-12-17T10:05:00Z"
       }
     ],
     total: 10
   }
   ↓
5. User clicks on a session
   ↓
6. Fetches GET /api/chat/sessions/{session_id}
   ↓
7. Loads all messages for that session
   ↓
8. User can continue conversation or view past messages
```

## Testing the Implementation

### Prerequisites

1. Backend server must be running and deployed
2. Database tables must be created (run `python3 -c "from backend.src.database import create_tables; create_tables()"`)
3. Frontend must be built and deployed

### Test Steps

1. **Create an account**
   - Go to the textbook website
   - Click "Sign Up" in header
   - Register with email and password

2. **Start a chat**
   - Click the 🤖 chat button (bottom right)
   - Type a message and send
   - Verify you get a response

3. **Check session is created**
   - Look for session_id in browser DevTools → Network tab → chat response
   - Should see: `{"response": "...", "session_id": "uuid-here"}`

4. **View history**
   - Click the History button (clock icon) in chat header
   - Should see your chat session listed
   - Session should show message count and timestamp

5. **Continue conversation**
   - Click on the session in history
   - Verify messages are loaded
   - Send another message
   - Verify it's added to the same session

6. **Test session persistence**
   - Close the chat widget
   - Reopen it
   - Should auto-load your last active session with messages

7. **Test delete**
   - Open history
   - Click delete (🗑️) on a session
   - Confirm deletion
   - Verify session is removed from list

8. **Test export**
   - Open history
   - Click export (📥) on a session
   - Verify a .txt file is downloaded with chat content

9. **Test clear all**
   - Open history
   - Click menu button (⋮)
   - Click "Clear All History"
   - Confirm
   - Verify all sessions are deleted

## Troubleshooting

### Issue: History button not visible

**Cause**: User not authenticated

**Solution**:
1. Check browser console for auth errors
2. Verify `localStorage.getItem('auth_token')` returns a token
3. Check AuthProvider is wrapping the app
4. Verify `/api/auth/get-session` returns user data

### Issue: Sessions not loading

**Cause**: CORS or authentication error

**Solution**:
1. Check browser console for CORS errors
2. Verify backend CORS settings include frontend URL
3. Check `/api/chat/sessions` endpoint with curl:
   ```bash
   curl -H "Authorization: Bearer YOUR_TOKEN" \
     https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/chat/sessions
   ```
4. Verify JWT token is valid (not expired)

### Issue: Messages not saving

**Cause**: Chat endpoint not receiving session_id or user not authenticated

**Solution**:
1. Check `/chat` request payload includes `session_id`
2. Verify Authorization header is present in request
3. Check backend logs for errors
4. Verify database connection is working

### Issue: "No chat history yet" shows but you've chatted

**Cause**: Anonymous chat or session not saved

**Solution**:
1. Verify you're logged in (check header shows your name)
2. Check that `/chat` response includes `session_id`
3. Query database directly:
   ```sql
   SELECT * FROM chat_sessions WHERE user_id = 'your-user-id';
   SELECT * FROM chat_messages WHERE session_id = 'session-uuid';
   ```
4. Check backend logs for database errors

### Issue: Database tables don't exist

**Cause**: Tables not created during initialization

**Solution**:
```bash
cd /path/to/hackathon-1
python3 -c "from backend.src.database import create_tables; create_tables()"
```

## Database Verification

To manually check if chat history is being stored:

```sql
-- Connect to Neon Postgres database

-- Check if tables exist
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public'
AND table_name IN ('chat_sessions', 'chat_messages');

-- View all sessions
SELECT * FROM chat_sessions ORDER BY updated_at DESC LIMIT 10;

-- View messages for a specific session
SELECT * FROM chat_messages
WHERE session_id = 'your-session-id'
ORDER BY created_at ASC;

-- Count messages per session
SELECT s.id, s.title, s.user_id, COUNT(m.id) as message_count
FROM chat_sessions s
LEFT JOIN chat_messages m ON s.id = m.session_id
GROUP BY s.id
ORDER BY s.updated_at DESC;
```

## Configuration

### Environment Variables

Backend requires these environment variables:

```env
DATABASE_URL=postgresql://...  # Neon Postgres connection string
BETTER_AUTH_SECRET=your-secret  # JWT signing key (must be set, no fallback)
OPENAI_API_KEY=sk-...  # OpenAI API key for RAG
QDRANT_URL=https://...  # Qdrant vector database URL
QDRANT_API_KEY=...  # Qdrant API key
```

### Frontend Configuration

Frontend URLs are hardcoded in components (should be env vars in production):

- Backend API: `https://physical-ai-and-humanoid-robotics-production.up.railway.app`
- Auth endpoints: `/api/auth/*`
- Chat endpoints: `/api/chat/*`

## Implementation Status

✅ **Completed Tasks (Phase 10)**:
- T147-T149: Database schema and models
- T150-T157: Chat service layer
- T158-T164: Chat API endpoints
- T165-T167: Context-aware RAG enhancement
- T168-T177: Frontend chat history UI
- T178-T180: Authentication integration
- T181-T185: Performance and UX polish

All 185 tasks in the project are complete, including the full chat history implementation.

## Key Features

1. **Automatic Session Management**
   - Auto-creates session on first message
   - Auto-loads last active session
   - Only one active session at a time

2. **Context-Aware Responses**
   - Includes last 10 messages as context for RAG
   - Better continuity in conversations

3. **Privacy & Security**
   - All endpoints require authentication
   - Ownership checks prevent accessing others' chats
   - JWT tokens with expiration
   - HTTPS only

4. **User Experience**
   - Pagination for large history
   - Real-time updates
   - Keyboard shortcuts (Ctrl+N, Ctrl+H)
   - Mobile responsive
   - Loading states and error handling

## Next Steps (If Issues Persist)

1. **Enable Debug Logging**
   ```python
   # In backend/main.py, add:
   import logging
   logging.basicConfig(level=logging.DEBUG)
   ```

2. **Check Browser Console**
   - Open DevTools (F12)
   - Go to Console tab
   - Look for errors related to chat or auth

3. **Check Network Tab**
   - Open DevTools (F12)
   - Go to Network tab
   - Filter by "chat" or "api"
   - Check request/response payloads

4. **Test Backend Directly**
   ```bash
   # Register a user
   curl -X POST https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth/sign-up \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","name":"Test","password":"test123","proficiency":"beginner"}'

   # Login and get token
   curl -X POST https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth/sign-in/email \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"test123"}'

   # Send a chat message (saves to session)
   curl -X POST https://physical-ai-and-humanoid-robotics-production.up.railway.app/chat \
     -H "Content-Type: application/json" \
     -H "Authorization: Bearer YOUR_TOKEN" \
     -d '{"query":"What is ROS 2?","selected_text":""}'

   # List sessions
   curl https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/chat/sessions \
     -H "Authorization: Bearer YOUR_TOKEN"
   ```

## Contact & Support

If chat history is still not working after following this guide:

1. Check the implementation files:
   - Backend: `backend/main.py`, `backend/src/services/chat_service.py`, `backend/src/models/chat.py`, `backend/src/database.py`
   - Frontend: `frontend/src/components/ChatWidget/index.js`, `frontend/src/components/ChatWidget/ChatHistory.js`

2. Review recent commits related to Phase 10 (chat history)

3. Check the tasks file: `specs/001-ai-textbook-physical-ai/tasks.md` (Tasks T147-T185)

4. Verify database schema matches the SQL in `backend/src/database.py` lines 109-158
