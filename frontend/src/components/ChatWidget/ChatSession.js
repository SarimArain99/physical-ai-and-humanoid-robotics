import React, { useState, useEffect } from "react";

/**
 * ChatSession Component - T169
 *
 * Displays messages in a specific chat session.
 * Features:
 * - Shows message history from a session
 * - Displays user and assistant messages with distinct styling
 * - Shows selected text context when available
 * - Provides back navigation to session list
 * - Auto-scrolls to bottom on new messages
 * - Shows loading and error states
 */
const ChatSession = ({ sessionId, onBack, onMessagesLoaded }) => {
  const [session, setSession] = useState(null);
  const [messages, setMessages] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // Load session when component mounts or sessionId changes
  useEffect(() => {
    if (sessionId) {
      loadSession();
    }
  }, [sessionId]);

  // Notify parent when messages are loaded
  useEffect(() => {
    if (messages.length > 0 && onMessagesLoaded) {
      onMessagesLoaded(messages);
    }
  }, [messages, onMessagesLoaded]);

  /**
   * Load session details and messages from backend API
   */
  const loadSession = async () => {
    setLoading(true);
    setError(null);

    try {
      const token = localStorage.getItem("auth_token");
      if (!token) {
        throw new Error("Authentication required");
      }

      const response = await fetch(
        `https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/chat/sessions/${sessionId}`,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            "Content-Type": "application/json",
          },
        }
      );

      if (!response.ok) {
        if (response.status === 404) {
          throw new Error("Session not found");
        } else if (response.status === 403) {
          throw new Error("Access denied");
        }
        throw new Error(`Failed to load session: ${response.status}`);
      }

      const data = await response.json();
      setSession(data.session);
      setMessages(data.messages || []);
    } catch (err) {
      console.error("Error loading session:", err);
      setError(err.message || "Failed to load session");
    } finally {
      setLoading(false);
    }
  };

  /**
   * Format timestamp for display
   */
  const formatTime = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  };

  /**
   * Get role display name
   */
  const getRoleDisplayName = (role) => {
    switch (role.toLowerCase()) {
      case "user":
        return "You";
      case "assistant":
        return "AI Assistant";
      case "system":
        return "System";
      default:
        return role;
    }
  };

  return (
    <div className="chat-session-container">
      {/* Header */}
      <div className="session-header">
        <button className="back-btn" onClick={onBack} title="Back to chat list">
          ← Back
        </button>
        <div className="session-info">
          {session && (
            <>
              <h3 className="session-title">{session.title}</h3>
              <span className="message-count">
                {messages.length}{" "}
                {messages.length === 1 ? "message" : "messages"}
              </span>
            </>
          )}
        </div>
      </div>

      {/* Loading State */}
      {loading && (
        <div className="session-loading">
          <div className="spinner"></div>
          <p>Loading session...</p>
        </div>
      )}

      {/* Error State */}
      {error && (
        <div className="session-error">
          <p>{error}</p>
          <div className="error-actions">
            <button onClick={loadSession}>Retry</button>
            <button onClick={onBack}>Go Back</button>
          </div>
        </div>
      )}

      {/* Messages */}
      {!loading && !error && messages.length > 0 && (
        <div className="messages-container">
          {messages.map((message, index) => (
            <div
              key={message.id || index}
              className={`message-row ${message.role.toLowerCase()}`}
            >
              {/* Message Avatar */}
              {message.role.toLowerCase() === "assistant" && (
                <div className="message-avatar">
                  <svg
                    width="20"
                    height="20"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                  >
                    <rect x="3" y="11" width="18" height="10" rx="2"></rect>
                    <circle cx="12" cy="5" r="2"></circle>
                    <path d="M12 7v4"></path>
                  </svg>
                </div>
              )}

              {/* Message Content */}
              <div className="message-content-wrapper">
                <div className="message-header">
                  <span className="message-role">
                    {getRoleDisplayName(message.role)}
                  </span>
                  <span className="message-time">
                    {formatTime(message.created_at)}
                  </span>
                </div>

                {/* Selected Text Context */}
                {message.selected_text && (
                  <div className="message-context">
                    <div className="context-label">📝 Context:</div>
                    <div className="context-text">
                      "{message.selected_text}"
                    </div>
                  </div>
                )}

                {/* Message Bubble */}
                <div className={`message-bubble ${message.role.toLowerCase()}`}>
                  {message.content}
                </div>

                {/* Sources (if available) */}
                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    <div className="sources-label">📚 Sources:</div>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx}>{source}</li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Empty State */}
      {!loading && !error && messages.length === 0 && (
        <div className="session-empty">
          <div className="empty-icon">💬</div>
          <p>No messages in this session</p>
        </div>
      )}

      {/* Styles */}
      <style jsx>{`
        .chat-session-container {
          display: flex;
          flex-direction: column;
          height: 100%;
          background-color: #151e29;
          color: #e2e8f0;
        }

        .session-header {
          padding: 18px 20px;
          background-color: #1e2a38;
          border-bottom: 1px solid #2c3e50;
          display: flex;
          align-items: center;
          gap: 16px;
        }

        .back-btn {
          background-color: #2c3e50;
          border: none;
          border-radius: 8px;
          padding: 8px 16px;
          color: #e2e8f0;
          font-weight: 600;
          cursor: pointer;
          transition: background-color 0.2s;
        }

        .back-btn:hover {
          background-color: #34495e;
        }

        .session-info {
          flex: 1;
          min-width: 0;
        }

        .session-title {
          margin: 0 0 4px 0;
          font-size: 16px;
          font-weight: 700;
          overflow: hidden;
          text-overflow: ellipsis;
          white-space: nowrap;
        }

        .message-count {
          font-size: 12px;
          color: #94a3b8;
        }

        .session-loading,
        .session-error,
        .session-empty {
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          padding: 40px 20px;
          text-align: center;
          flex: 1;
        }

        .spinner {
          width: 40px;
          height: 40px;
          border: 3px solid #2c3e50;
          border-top-color: #2ecc71;
          border-radius: 50%;
          animation: spin 1s linear infinite;
        }

        @keyframes spin {
          to {
            transform: rotate(360deg);
          }
        }

        .session-loading p {
          margin-top: 16px;
          color: #94a3b8;
        }

        .session-error {
          color: #ef4444;
        }

        .error-actions {
          display: flex;
          gap: 12px;
          margin-top: 16px;
        }

        .error-actions button {
          padding: 8px 20px;
          background-color: #2ecc71;
          border: none;
          border-radius: 8px;
          color: #1e2a38;
          font-weight: 600;
          cursor: pointer;
        }

        .error-actions button:last-child {
          background-color: #2c3e50;
          color: #e2e8f0;
        }

        .empty-icon {
          font-size: 48px;
          margin-bottom: 16px;
          opacity: 0.5;
        }

        .session-empty p {
          color: #94a3b8;
        }

        .messages-container {
          flex: 1;
          overflow-y: auto;
          padding: 20px;
          display: flex;
          flex-direction: column;
          gap: 20px;
        }

        .message-row {
          display: flex;
          gap: 12px;
          align-items: flex-start;
          animation: fadeIn 0.3s ease;
        }

        .message-row.user {
          flex-direction: row-reverse;
        }

        @keyframes fadeIn {
          from {
            opacity: 0;
            transform: translateY(10px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }

        .message-avatar {
          width: 32px;
          height: 32px;
          background-color: #2c3e50;
          border-radius: 50%;
          display: flex;
          align-items: center;
          justify-content: center;
          color: #2ecc71;
          flex-shrink: 0;
        }

        .message-content-wrapper {
          flex: 1;
          min-width: 0;
          display: flex;
          flex-direction: column;
          gap: 8px;
        }

        .message-row.user .message-content-wrapper {
          align-items: flex-end;
        }

        .message-header {
          display: flex;
          gap: 12px;
          align-items: center;
          font-size: 12px;
        }

        .message-role {
          font-weight: 600;
          color: #94a3b8;
        }

        .message-time {
          color: #64748b;
        }

        .message-context {
          background-color: #2c3e50;
          padding: 10px 14px;
          border-radius: 8px;
          border-left: 3px solid #2ecc71;
          font-size: 13px;
        }

        .context-label {
          font-weight: 600;
          margin-bottom: 6px;
          color: #a8d5ff;
        }

        .context-text {
          color: #cbd5e1;
          font-style: italic;
        }

        .message-bubble {
          padding: 12px 16px;
          border-radius: 18px;
          max-width: 85%;
          line-height: 1.5;
          font-size: 14px;
          box-shadow: 0 1px 2px rgba(0, 0, 0, 0.2);
          word-wrap: break-word;
        }

        .message-bubble.assistant {
          background-color: #2c3e50;
          color: #e2e8f0;
          border-bottom-left-radius: 4px;
        }

        .message-bubble.user {
          background-color: #2ecc71;
          color: #1e2a38;
          border-bottom-right-radius: 4px;
          font-weight: 500;
        }

        .message-bubble.system {
          background-color: #475569;
          color: #e2e8f0;
          font-style: italic;
          font-size: 13px;
        }

        .message-sources {
          background-color: #1e2a38;
          padding: 10px 14px;
          border-radius: 8px;
          font-size: 12px;
        }

        .sources-label {
          font-weight: 600;
          margin-bottom: 6px;
          color: #94a3b8;
        }

        .message-sources ul {
          margin: 0;
          padding-left: 20px;
          color: #cbd5e1;
        }

        .message-sources li {
          margin: 4px 0;
        }

        /* Scrollbar styling */
        .messages-container::-webkit-scrollbar {
          width: 6px;
        }

        .messages-container::-webkit-scrollbar-track {
          background: #151e29;
        }

        .messages-container::-webkit-scrollbar-thumb {
          background: #2c3e50;
          border-radius: 3px;
        }

        .messages-container::-webkit-scrollbar-thumb:hover {
          background: #34495e;
        }
      `}</style>
    </div>
  );
};

export default ChatSession;
