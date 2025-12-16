"""
Chat models for stateful conversation history (Phase 10).

T147: ChatSession model - Represents a conversation session for a user
T148: ChatMessage model - Represents individual messages within a session
"""
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, List
from enum import Enum


class MessageRole(str, Enum):
    """Role of the message sender."""
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"


@dataclass
class ChatSession:
    """
    Represents a chat conversation session.

    Attributes:
        id: Unique session identifier (UUID)
        user_id: Owner of the session (from users table)
        title: Display title (auto-generated or user-set)
        created_at: When the session was created
        updated_at: When the session was last updated
        is_active: Whether this is the user's current active session
        message_count: Number of messages in session (for display)
    """
    id: str
    user_id: str
    title: str
    created_at: datetime
    updated_at: datetime
    is_active: bool = True
    message_count: int = 0

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON response."""
        return {
            "id": self.id,
            "user_id": self.user_id,
            "title": self.title,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
            "is_active": self.is_active,
            "message_count": self.message_count
        }

    @classmethod
    def from_db_row(cls, row: dict) -> "ChatSession":
        """Create instance from database row."""
        return cls(
            id=row["id"],
            user_id=row["user_id"],
            title=row["title"],
            created_at=row["created_at"],
            updated_at=row["updated_at"],
            is_active=row.get("is_active", True),
            message_count=row.get("message_count", 0)
        )


@dataclass
class ChatMessage:
    """
    Represents a single message in a chat session.

    Attributes:
        id: Unique message identifier (UUID)
        session_id: Parent session ID
        role: Who sent the message (user/assistant/system)
        content: The message text content
        selected_text: Optional text that was highlighted when asking (for context)
        created_at: When the message was sent
        sources: Optional list of sources cited in response (for assistant messages)
    """
    id: str
    session_id: str
    role: MessageRole
    content: str
    created_at: datetime
    selected_text: Optional[str] = None
    sources: Optional[List[str]] = None

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON response."""
        return {
            "id": self.id,
            "session_id": self.session_id,
            "role": self.role.value if isinstance(self.role, MessageRole) else self.role,
            "content": self.content,
            "selected_text": self.selected_text,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "sources": self.sources
        }

    @classmethod
    def from_db_row(cls, row: dict) -> "ChatMessage":
        """Create instance from database row."""
        role = row["role"]
        if isinstance(role, str):
            role = MessageRole(role)

        return cls(
            id=row["id"],
            session_id=row["session_id"],
            role=role,
            content=row["content"],
            selected_text=row.get("selected_text"),
            created_at=row["created_at"],
            sources=row.get("sources")
        )


@dataclass
class ChatSessionWithMessages:
    """
    A session with its messages loaded (for session detail view).
    """
    session: ChatSession
    messages: List[ChatMessage]

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON response."""
        return {
            "session": self.session.to_dict(),
            "messages": [msg.to_dict() for msg in self.messages]
        }
