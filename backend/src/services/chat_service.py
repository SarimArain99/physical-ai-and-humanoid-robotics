"""
ChatService - Business logic for chat session management (Phase 10, T150-T157).

Handles CRUD operations for chat sessions and messages with proper auth checks.
"""
import uuid
import json
from datetime import datetime, timezone
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import text

from src.models.chat import ChatSession, ChatMessage, ChatSessionWithMessages, MessageRole


class ChatService:
    """Service layer for chat session and message management."""

    def __init__(self, db: Session):
        """Initialize with database session."""
        self.db = db

    # --- T151: create_session ---
    def create_session(self, user_id: str, title: Optional[str] = None) -> ChatSession:
        """
        Create a new chat session for a user.

        Args:
            user_id: User ID from JWT token
            title: Optional custom title (default: "New Chat")

        Returns:
            Created ChatSession

        Raises:
            Exception: If database operation fails
        """
        session_id = str(uuid.uuid4())
        now = datetime.now(timezone.utc)
        title = title or "New Chat"

        # Deactivate other sessions for this user (only one active at a time)
        self.db.execute(
            text("UPDATE chat_sessions SET is_active = FALSE WHERE user_id = :user_id"),
            {"user_id": user_id}
        )

        # Create new session
        self.db.execute(
            text("""
                INSERT INTO chat_sessions (id, user_id, title, is_active, created_at, updated_at)
                VALUES (:id, :user_id, :title, TRUE, :now, :now)
            """),
            {
                "id": session_id,
                "user_id": user_id,
                "title": title,
                "now": now
            }
        )
        self.db.commit()

        return ChatSession(
            id=session_id,
            user_id=user_id,
            title=title,
            created_at=now,
            updated_at=now,
            is_active=True,
            message_count=0
        )

    # --- T152: get_user_sessions ---
    def get_user_sessions(
        self,
        user_id: str,
        limit: int = 20,
        offset: int = 0
    ) -> List[ChatSession]:
        """
        Get paginated list of user's chat sessions, ordered by most recent.

        Args:
            user_id: User ID from JWT token
            limit: Maximum number of sessions to return (default 20)
            offset: Pagination offset (default 0)

        Returns:
            List of ChatSession objects
        """
        result = self.db.execute(
            text("""
                SELECT s.*,
                       COUNT(m.id) as message_count
                FROM chat_sessions s
                LEFT JOIN chat_messages m ON s.id = m.session_id
                WHERE s.user_id = :user_id
                GROUP BY s.id
                ORDER BY s.updated_at DESC
                LIMIT :limit OFFSET :offset
            """),
            {"user_id": user_id, "limit": limit, "offset": offset}
        )

        sessions = []
        for row in result.mappings():
            session = ChatSession.from_db_row(dict(row))
            sessions.append(session)

        return sessions

    # --- T153: get_session_messages ---
    def get_session_messages(
        self,
        session_id: str,
        user_id: str
    ) -> ChatSessionWithMessages:
        """
        Get a session with all its messages (with auth check).

        Args:
            session_id: Session ID to retrieve
            user_id: User ID from JWT token (for ownership check)

        Returns:
            ChatSessionWithMessages object

        Raises:
            PermissionError: If user doesn't own the session
            ValueError: If session not found
        """
        # Get session with ownership check
        session_result = self.db.execute(
            text("SELECT * FROM chat_sessions WHERE id = :id AND user_id = :user_id"),
            {"id": session_id, "user_id": user_id}
        )
        session_row = session_result.mappings().fetchone()

        if not session_row:
            raise ValueError(f"Session {session_id} not found or access denied")

        # Get messages for session
        messages_result = self.db.execute(
            text("""
                SELECT * FROM chat_messages
                WHERE session_id = :session_id
                ORDER BY created_at ASC
            """),
            {"session_id": session_id}
        )

        messages = []
        for msg_row in messages_result.mappings():
            messages.append(ChatMessage.from_db_row(dict(msg_row)))

        session = ChatSession.from_db_row(dict(session_row))
        session.message_count = len(messages)

        return ChatSessionWithMessages(session=session, messages=messages)

    # --- T154: add_message ---
    def add_message(
        self,
        session_id: str,
        role: MessageRole,
        content: str,
        selected_text: Optional[str] = None,
        sources: Optional[List[str]] = None
    ) -> ChatMessage:
        """
        Add a message to a session.

        Args:
            session_id: Session to add message to
            role: Message role (user/assistant/system)
            content: Message content
            selected_text: Optional highlighted text (for user messages)
            sources: Optional list of sources (for assistant messages)

        Returns:
            Created ChatMessage

        Raises:
            Exception: If database operation fails
        """
        message_id = str(uuid.uuid4())
        now = datetime.now(timezone.utc)

        # Convert sources list to JSON string for storage
        sources_json = json.dumps(sources) if sources else None

        # Insert message
        self.db.execute(
            text("""
                INSERT INTO chat_messages (id, session_id, role, content, selected_text, sources, created_at)
                VALUES (:id, :session_id, :role, :content, :selected_text, :sources, :now)
            """),
            {
                "id": message_id,
                "session_id": session_id,
                "role": role.value if isinstance(role, MessageRole) else role,
                "content": content,
                "selected_text": selected_text,
                "sources": sources_json,
                "now": now
            }
        )

        # Update session updated_at timestamp
        self.db.execute(
            text("UPDATE chat_sessions SET updated_at = :now WHERE id = :session_id"),
            {"now": now, "session_id": session_id}
        )

        self.db.commit()

        return ChatMessage(
            id=message_id,
            session_id=session_id,
            role=role,
            content=content,
            selected_text=selected_text,
            sources=sources,
            created_at=now
        )

    # --- T155: delete_session ---
    def delete_session(self, session_id: str, user_id: str) -> bool:
        """
        Delete a chat session (with auth check).

        Args:
            session_id: Session ID to delete
            user_id: User ID from JWT token (for ownership check)

        Returns:
            True if deleted, False if not found

        Raises:
            PermissionError: If user doesn't own the session
        """
        # Check ownership before deleting
        result = self.db.execute(
            text("SELECT user_id FROM chat_sessions WHERE id = :id"),
            {"id": session_id}
        )
        row = result.fetchone()

        if not row:
            return False

        if row[0] != user_id:
            raise PermissionError(f"User {user_id} does not own session {session_id}")

        # Delete session (CASCADE will delete messages automatically)
        self.db.execute(
            text("DELETE FROM chat_sessions WHERE id = :id"),
            {"id": session_id}
        )
        self.db.commit()

        return True

    # --- T156: clear_all_history ---
    def clear_all_history(self, user_id: str) -> int:
        """
        Clear all chat history for a user.

        Args:
            user_id: User ID from JWT token

        Returns:
            Number of sessions deleted
        """
        # Count sessions before deleting
        count_result = self.db.execute(
            text("SELECT COUNT(*) FROM chat_sessions WHERE user_id = :user_id"),
            {"user_id": user_id}
        )
        count = count_result.scalar()

        # Delete all user's sessions (CASCADE deletes messages)
        self.db.execute(
            text("DELETE FROM chat_sessions WHERE user_id = :user_id"),
            {"user_id": user_id}
        )
        self.db.commit()

        return count

    # --- T157: export_session ---
    def export_session(
        self,
        session_id: str,
        user_id: str,
        format: str = "text"
    ) -> str:
        """
        Export a session as text or JSON.

        Args:
            session_id: Session ID to export
            user_id: User ID from JWT token (for ownership check)
            format: Export format ('text' or 'json')

        Returns:
            Exported content as string

        Raises:
            PermissionError: If user doesn't own the session
            ValueError: If session not found or invalid format
        """
        # Get session with messages
        session_with_messages = self.get_session_messages(session_id, user_id)

        if format == "json":
            return json.dumps(session_with_messages.to_dict(), indent=2)

        elif format == "text":
            lines = [
                f"Chat Session: {session_with_messages.session.title}",
                f"Created: {session_with_messages.session.created_at.strftime('%Y-%m-%d %H:%M:%S')}",
                f"Messages: {len(session_with_messages.messages)}",
                "\n" + "="*60 + "\n"
            ]

            for msg in session_with_messages.messages:
                role_label = msg.role.value.upper() if isinstance(msg.role, MessageRole) else msg.role.upper()
                timestamp = msg.created_at.strftime("%Y-%m-%d %H:%M:%S")

                lines.append(f"[{timestamp}] {role_label}:")

                if msg.selected_text:
                    lines.append(f"  (Re: \"{msg.selected_text[:50]}...\")")

                lines.append(f"  {msg.content}\n")

            return "\n".join(lines)

        else:
            raise ValueError(f"Invalid export format: {format}. Use 'text' or 'json'.")

    # --- Utility Methods ---

    def get_active_session(self, user_id: str) -> Optional[ChatSession]:
        """
        Get the user's currently active session (if any).

        Args:
            user_id: User ID from JWT token

        Returns:
            ChatSession or None if no active session
        """
        result = self.db.execute(
            text("""
                SELECT s.*,
                       COUNT(m.id) as message_count
                FROM chat_sessions s
                LEFT JOIN chat_messages m ON s.id = m.session_id
                WHERE s.user_id = :user_id AND s.is_active = TRUE
                GROUP BY s.id
                LIMIT 1
            """),
            {"user_id": user_id}
        )
        row = result.mappings().fetchone()

        return ChatSession.from_db_row(dict(row)) if row else None

    def update_session_title(self, session_id: str, user_id: str, title: str) -> bool:
        """
        Update session title (with auth check).

        Args:
            session_id: Session ID to update
            user_id: User ID from JWT token
            title: New title

        Returns:
            True if updated, False if not found

        Raises:
            PermissionError: If user doesn't own the session
        """
        # Check ownership
        result = self.db.execute(
            text("SELECT user_id FROM chat_sessions WHERE id = :id"),
            {"id": session_id}
        )
        row = result.fetchone()

        if not row:
            return False

        if row[0] != user_id:
            raise PermissionError(f"User {user_id} does not own session {session_id}")

        # Update title
        self.db.execute(
            text("UPDATE chat_sessions SET title = :title, updated_at = :now WHERE id = :id"),
            {"title": title, "now": datetime.now(timezone.utc), "id": session_id}
        )
        self.db.commit()

        return True
