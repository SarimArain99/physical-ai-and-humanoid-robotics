import React, { useState, useEffect } from 'react';

/**
 * ChatHistory Component - T168
 *
 * Displays a list of past chat sessions for authenticated users.
 * Features:
 * - Loads sessions from backend API
 * - Shows session title and message count
 * - Allows selecting a session to view/continue
 * - Shows loading and empty states
 * - Highlights currently active session
 */
const ChatHistory = ({
  onSelectSession,
  onNewChat,
  currentSessionId,
  onClose,
  onDeleteSession,
  onExportSession,
  onClearAll,
  user, // T190: Accept user prop from parent
  authLoading // T195: Accept auth loading state
}) => {
  const [sessions, setSessions] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [showMenu, setShowMenu] = useState(false);
  // T181: Pagination state
  const [currentPage, setCurrentPage] = useState(1);
  const [hasMore, setHasMore] = useState(false);
  const [totalSessions, setTotalSessions] = useState(0);
  const sessionsPerPage = 10;

  // T208: Load sessions when component mounts, page changes, or auth state changes
  useEffect(() => {
    // T209: Don't load if auth is still loading
    if (authLoading) {
      return;
    }

    // Only load if user is authenticated
    if (user) {
      loadSessions();
    }
  }, [currentPage, user, authLoading]); // T208: Added user and authLoading to dependencies

  /**
   * Load chat sessions from backend API with pagination
   */
  const loadSessions = async () => {
    setLoading(true);
    setError(null);

    try {
      // T195: Don't show error if auth is still loading
      if (authLoading) {
        setLoading(false);
        return;
      }

      // T192: Check user from AuthProvider first
      if (!user) {
        setError('Please sign in to view chat history');
        setLoading(false);
        return;
      }

      // T193: Get token from localStorage with retry logic
      // If user exists, AuthProvider has validated the token
      let token = typeof window !== 'undefined' && typeof localStorage !== 'undefined'
        ? localStorage.getItem('auth_token')
        : null;

      // T210: Retry token fetch after a brief delay if not found
      // This handles race conditions where user is set before token is stored
      if (!token && user) {
        await new Promise(resolve => setTimeout(resolve, 100));
        token = localStorage.getItem('auth_token');
      }

      if (!token) {
        // This shouldn't happen if AuthProvider is working correctly
        // But if it does, show a clear error
        console.error('ChatHistory: No token despite user being authenticated', {
          user: user?.email || user?.name,
          userObject: user,
          tokenAttempts: 2
        });
        setError('Authentication error. Please try signing out and back in.');
        setLoading(false);
        return;
      }

      // T181: Add pagination parameters
      const offset = (currentPage - 1) * sessionsPerPage;
      const response = await fetch(
        `https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/chat/sessions?limit=${sessionsPerPage}&offset=${offset}`,
        {
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );

      if (!response.ok) {
        throw new Error(`Failed to load sessions: ${response.status}`);
      }

      const data = await response.json();
      setSessions(data.sessions || []);
      setTotalSessions(data.total || 0);
      setHasMore((data.sessions || []).length === sessionsPerPage);
    } catch (err) {
      console.error('Error loading sessions:', err);
      setError('Failed to load chat history');
    } finally {
      setLoading(false);
    }
  };

  /**
   * Handle session selection
   */
  const handleSelectSession = (sessionId) => {
    onSelectSession(sessionId);
    if (onClose) onClose(); // Close sidebar on mobile after selection
  };

  /**
   * Handle new chat button
   */
  const handleNewChat = () => {
    onNewChat();
    if (onClose) onClose(); // Close sidebar after action
  };

  /**
   * Format date for display
   */
  const formatDate = (dateString) => {
    const date = new Date(dateString);
    const now = new Date();
    const diffMs = now - date;
    const diffMins = Math.floor(diffMs / 60000);
    const diffHours = Math.floor(diffMs / 3600000);
    const diffDays = Math.floor(diffMs / 86400000);

    if (diffMins < 1) return 'Just now';
    if (diffMins < 60) return `${diffMins}m ago`;
    if (diffHours < 24) return `${diffHours}h ago`;
    if (diffDays < 7) return `${diffDays}d ago`;
    return date.toLocaleDateString();
  };

  return (
    <div className="chat-history-container">
      {/* Header */}
      <div className="history-header">
        <h3>Chat History</h3>
        <div className="header-actions">
          <button className="new-chat-btn" onClick={handleNewChat} title="Start new chat">
            + New
          </button>
          {/* T176: Clear All History button */}
          {onClearAll && sessions.length > 0 && (
            <button
              className="menu-btn"
              onClick={() => setShowMenu(!showMenu)}
              title="More options"
            >
              ⋮
            </button>
          )}
        </div>
      </div>

      {/* T176: Dropdown menu for Clear All */}
      {showMenu && (
        <div className="history-menu">
          <button
            className="menu-item danger"
            onClick={async () => {
              setShowMenu(false);
              if (onClearAll) {
                const success = await onClearAll();
                if (success) loadSessions();
              }
            }}
          >
            🗑️ Clear All History
          </button>
        </div>
      )}

      {/* Loading State */}
      {loading && (
        <div className="history-loading">
          <div className="spinner"></div>
          <p>Loading history...</p>
        </div>
      )}

      {/* Error State */}
      {error && (
        <div className="history-error">
          <p>{error}</p>
          <button onClick={loadSessions}>Retry</button>
        </div>
      )}

      {/* Empty State */}
      {!loading && !error && sessions.length === 0 && (
        <div className="history-empty">
          <div className="empty-icon">💬</div>
          <p>No chat history yet</p>
          <p className="empty-subtitle">Start a conversation to see it here</p>
        </div>
      )}

      {/* Sessions List */}
      {!loading && !error && sessions.length > 0 && (
        <div className="sessions-list">
          {sessions.map((session) => (
            <div
              key={session.id}
              className={`session-item ${session.id === currentSessionId ? 'active' : ''}`}
            >
              <div
                className="session-clickable"
                onClick={() => handleSelectSession(session.id)}
              >
                <div className="session-content">
                  <div className="session-title">{session.title}</div>
                  <div className="session-meta">
                    <span className="message-count">
                      {session.message_count} {session.message_count === 1 ? 'message' : 'messages'}
                    </span>
                    <span className="session-date">
                      {formatDate(session.updated_at)}
                    </span>
                  </div>
                </div>
                {session.is_active && (
                  <div className="active-indicator" title="Active session">●</div>
                )}
              </div>

              {/* T175 & T177: Action buttons for delete and export */}
              <div className="session-actions">
                {onExportSession && (
                  <button
                    className="action-btn export-btn"
                    onClick={(e) => {
                      e.stopPropagation();
                      onExportSession(session.id, 'text');
                    }}
                    title="Export chat"
                  >
                    📥
                  </button>
                )}
                {onDeleteSession && (
                  <button
                    className="action-btn delete-btn"
                    onClick={async (e) => {
                      e.stopPropagation();
                      const success = await onDeleteSession(session.id);
                      if (success) loadSessions();
                    }}
                    title="Delete chat"
                  >
                    🗑️
                  </button>
                )}
              </div>
            </div>
          ))}

          {/* T181: Pagination controls */}
          {sessions.length > 0 && (
            <div className="pagination-controls">
              <button
                className="pagination-btn"
                onClick={() => setCurrentPage(prev => Math.max(1, prev - 1))}
                disabled={currentPage === 1}
              >
                ← Previous
              </button>
              <span className="pagination-info">
                Page {currentPage} {totalSessions > 0 && `(${totalSessions} total)`}
              </span>
              <button
                className="pagination-btn"
                onClick={() => setCurrentPage(prev => prev + 1)}
                disabled={!hasMore}
              >
                Next →
              </button>
            </div>
          )}
        </div>
      )}

      {/* Styles */}
      <style jsx>{`
        .chat-history-container {
          display: flex;
          flex-direction: column;
          height: 100%;
          background-color: #1e2a38;
          color: #e2e8f0;
        }

        .history-header {
          padding: 18px 20px;
          background: linear-gradient(90deg, #2ecc71, #27ae60);
          color: #1e2a38;
          display: flex;
          justify-content: space-between;
          align-items: center;
          border-bottom: 1px solid #2c3e50;
        }

        .history-header h3 {
          margin: 0;
          font-size: 16px;
          font-weight: 700;
        }

        .header-actions {
          display: flex;
          gap: 8px;
          align-items: center;
        }

        .new-chat-btn {
          background-color: rgba(255, 255, 255, 0.2);
          border: none;
          border-radius: 8px;
          padding: 8px 16px;
          color: #1e2a38;
          font-weight: 600;
          cursor: pointer;
          transition: background-color 0.2s;
        }

        .new-chat-btn:hover {
          background-color: rgba(255, 255, 255, 0.3);
        }

        .menu-btn {
          background-color: rgba(255, 255, 255, 0.2);
          border: none;
          border-radius: 8px;
          padding: 8px 12px;
          color: #1e2a38;
          font-weight: 700;
          font-size: 18px;
          cursor: pointer;
          transition: background-color 0.2s;
          line-height: 1;
        }

        .menu-btn:hover {
          background-color: rgba(255, 255, 255, 0.3);
        }

        .history-menu {
          position: absolute;
          top: 60px;
          right: 20px;
          background-color: #2c3e50;
          border-radius: 8px;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
          z-index: 1000;
          min-width: 180px;
          overflow: hidden;
        }

        .menu-item {
          width: 100%;
          padding: 12px 16px;
          background: none;
          border: none;
          color: #e2e8f0;
          text-align: left;
          cursor: pointer;
          transition: background-color 0.2s;
          font-size: 14px;
          display: flex;
          align-items: center;
          gap: 8px;
        }

        .menu-item:hover {
          background-color: #34495e;
        }

        .menu-item.danger {
          color: #ef4444;
        }

        .menu-item.danger:hover {
          background-color: rgba(239, 68, 68, 0.1);
        }

        .history-loading,
        .history-error,
        .history-empty {
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
          to { transform: rotate(360deg); }
        }

        .history-loading p {
          margin-top: 16px;
          color: #94a3b8;
        }

        .history-error {
          color: #ef4444;
        }

        .history-error button {
          margin-top: 12px;
          padding: 8px 20px;
          background-color: #2ecc71;
          border: none;
          border-radius: 8px;
          color: #1e2a38;
          font-weight: 600;
          cursor: pointer;
        }

        .empty-icon {
          font-size: 48px;
          margin-bottom: 16px;
          opacity: 0.5;
        }

        .history-empty p {
          margin: 8px 0;
          color: #94a3b8;
        }

        .empty-subtitle {
          font-size: 14px;
          color: #64748b;
        }

        .sessions-list {
          flex: 1;
          overflow-y: auto;
          padding: 12px;
        }

        .session-item {
          display: flex;
          align-items: center;
          gap: 12px;
          padding: 14px 16px;
          margin-bottom: 8px;
          background-color: #2c3e50;
          border-radius: 12px;
          transition: all 0.2s;
          border: 2px solid transparent;
        }

        .session-item:hover {
          background-color: #34495e;
        }

        .session-item.active {
          background-color: #2ecc71;
          color: #1e2a38;
          border-color: #27ae60;
        }

        .session-clickable {
          flex: 1;
          display: flex;
          align-items: center;
          gap: 12px;
          cursor: pointer;
          min-width: 0;
        }

        .session-clickable:hover {
          transform: translateX(4px);
        }

        .session-content {
          flex: 1;
          min-width: 0;
        }

        .session-actions {
          display: flex;
          gap: 6px;
          flex-shrink: 0;
        }

        .action-btn {
          background-color: transparent;
          border: none;
          padding: 6px 10px;
          cursor: pointer;
          border-radius: 6px;
          font-size: 16px;
          transition: background-color 0.2s, transform 0.1s;
          opacity: 0.7;
        }

        .action-btn:hover {
          opacity: 1;
          transform: scale(1.1);
        }

        .export-btn:hover {
          background-color: rgba(46, 204, 113, 0.2);
        }

        .delete-btn:hover {
          background-color: rgba(239, 68, 68, 0.2);
        }

        .session-item.active .action-btn {
          opacity: 0.8;
        }

        .session-item.active .action-btn:hover {
          opacity: 1;
        }

        /* T181: Pagination styles */
        .pagination-controls {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 16px;
          background-color: #1e2a38;
          border-top: 1px solid #2c3e50;
          gap: 12px;
        }

        .pagination-btn {
          background-color: #2c3e50;
          border: none;
          border-radius: 8px;
          padding: 8px 16px;
          color: #e2e8f0;
          font-size: 13px;
          font-weight: 600;
          cursor: pointer;
          transition: background-color 0.2s;
        }

        .pagination-btn:hover:not(:disabled) {
          background-color: #34495e;
        }

        .pagination-btn:disabled {
          opacity: 0.4;
          cursor: not-allowed;
        }

        .pagination-info {
          font-size: 13px;
          color: #94a3b8;
          font-weight: 500;
        }

        .session-title {
          font-size: 14px;
          font-weight: 600;
          margin-bottom: 6px;
          overflow: hidden;
          text-overflow: ellipsis;
          white-space: nowrap;
        }

        .session-item.active .session-title {
          color: #1e2a38;
        }

        .session-meta {
          display: flex;
          gap: 12px;
          font-size: 12px;
          color: #94a3b8;
        }

        .session-item.active .session-meta {
          color: rgba(30, 42, 56, 0.7);
        }

        .message-count::before {
          content: '💬 ';
        }

        .session-date::before {
          content: '🕒 ';
        }

        .active-indicator {
          color: #2ecc71;
          font-size: 20px;
          animation: pulse-dot 2s ease-in-out infinite;
        }

        .session-item.active .active-indicator {
          color: #1e2a38;
        }

        @keyframes pulse-dot {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }

        /* Scrollbar styling */
        .sessions-list::-webkit-scrollbar {
          width: 6px;
        }

        .sessions-list::-webkit-scrollbar-track {
          background: #151e29;
        }

        .sessions-list::-webkit-scrollbar-thumb {
          background: #2c3e50;
          border-radius: 3px;
        }

        .sessions-list::-webkit-scrollbar-thumb:hover {
          background: #34495e;
        }
      `}</style>
    </div>
  );
};

export default ChatHistory;
