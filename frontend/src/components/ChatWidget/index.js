import React, { useState, useEffect, useRef } from "react";
import { useAuth } from "../Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";
import ChatHistory from "./ChatHistory";
import ChatSession from "./ChatSession";
import EnhancedChatIcon from "./ChatIcon.enhanced";
import "./ChatIcon.enhanced.css";

// Use EnhancedChatIcon as ChatIcon for consistency
const ChatIcon = EnhancedChatIcon;
import { API_BASE_URL, chatUrls } from "../../config/api";

// --- ICONS (SVG) ---
const SendIcon = () => (
  <svg
    width="20"
    height="20"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <line x1="22" y1="2" x2="11" y2="13"></line>
    <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
  </svg>
);
const CloseIcon = () => (
  <svg
    width="24"
    height="24"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <line x1="18" y1="6" x2="6" y2="18"></line>
    <line x1="6" y1="6" x2="18" y2="18"></line>
  </svg>
);
const HistoryIcon = () => (
  <svg
    width="20"
    height="20"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <circle cx="12" cy="12" r="10"></circle>
    <polyline points="12 6 12 12 16 14"></polyline>
  </svg>
);
const NewChatIcon = () => (
  <svg
    width="20"
    height="20"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
  >
    <line x1="12" y1="5" x2="12" y2="19"></line>
    <line x1="5" y1="12" x2="19" y2="12"></line>
  </svg>
);

const ChatWidgetContent = () => {
  const { user, loading: authLoading } = useAuth(); // T195: Get auth loading state
  const [isOpen, setIsOpen] = useState(false);
  const [authChecked, setAuthChecked] = useState(false); // T178: Track auth state check
  const [messages, setMessages] = useState([
    {
      role: "assistant",
      content:
        "Hello! I am your Physical AI assistant. How can I help you today? Tip: Select/highlight text on the page and ask me about it!",
    },
  ]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState("");
  const messagesEndRef = useRef(null);

  // Streaming typing effect state
  const [typingMessage, setTypingMessage] = useState(null);
  const [streamedContent, setStreamedContent] = useState("");

  // T170: Session management state
  const [currentSessionId, setCurrentSessionId] = useState(null);
  const [showHistory, setShowHistory] = useState(false);
  const [viewingSession, setViewingSession] = useState(false);

  // T182: Enhanced loading states
  const [operationLoading, setOperationLoading] = useState(null);

  // T184: Error handling with retry
  const [lastError, setLastError] = useState(null);
  const [retryCallback, setRetryCallback] = useState(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen, loading, streamedContent]);

  // Streaming typing effect hook
  useEffect(() => {
    if (!typingMessage) return;

    const fullText = typingMessage.content;
    let index = 0;
    const typingSpeed = 15; // ms per character

    const typeNextChar = () => {
      if (index < fullText.length) {
        setStreamedContent(fullText.slice(0, index + 1));
        index++;
        setTimeout(typeNextChar, typingSpeed);
      } else {
        // Done typing - add to messages and clear state
        setMessages((prev) => [...prev, typingMessage]);
        setTypingMessage(null);
        setStreamedContent("");
      }
    };

    typeNextChar();

    return () => {
      // Cleanup if component unmounts or message changes
      setStreamedContent("");
    };
  }, [typingMessage]);

  // T173: Continue Last Chat - Load active session when widget opens
  useEffect(() => {
    if (user && isOpen && !currentSessionId) {
      checkAndLoadActiveSession();
    }
  }, [user, isOpen]);

  const checkAndLoadActiveSession = async () => {
    try {
      let token = localStorage.getItem("auth_token");

      // T210: Retry token fetch if user exists but token not found (race condition)
      if (!token && user) {
        await new Promise((resolve) => setTimeout(resolve, 100));
        token = localStorage.getItem("auth_token");
      }

      if (!token) {
        console.warn("No auth token available for loading session");
        return;
      }

      const response = await fetch(chatUrls.listSessions(1), {
        headers: {
          Authorization: `Bearer ${token}`,
          "Content-Type": "application/json",
          },
        });

      if (response.ok) {
        const data = await response.json();
        if (data.sessions && data.sessions.length > 0) {
          const lastSession = data.sessions[0];
          // Only auto-load if it's the active session
          if (lastSession.is_active) {
            setCurrentSessionId(lastSession.id);
            // Load messages from session
            loadSessionMessages(lastSession.id);
          }
        }
      } else if (response.status === 404) {
        console.warn("Chat sessions endpoint not found (404)");
        // Endpoint might not be deployed yet, fail gracefully
      } else {
        console.warn("Failed to load active session:", response.status);
      }
    } catch (error) {
      console.error("Failed to check active session:", error);
      // Fail silently - user can manually access history
    }
  };

  const loadSessionMessages = async (sessionId) => {
    try {
      const token = localStorage.getItem("auth_token");
      const response = await fetch(chatUrls.getSession(sessionId), {
        headers: {
          Authorization: `Bearer ${token}`,
          "Content-Type": "application/json",
          },
        });

      if (response.ok) {
        const data = await response.json();
        if (data.messages && data.messages.length > 0) {
          // Convert API messages to chat format
          const chatMessages = data.messages.map((msg) => ({
            role: msg.role,
            content: msg.content,
            selected_text: msg.selected_text,
          }));
          setMessages(chatMessages);
        }
      }
    } catch (error) {
      console.error("Failed to load session messages:", error);
    }
  };

  // Capture text selection from the page (FR-005)
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || "";
      // Only capture if text is from the main content area (not from chat widget)
      if (text && text.length > 0 && text.length < 5000) {
        const anchorNode = selection?.anchorNode;
        // Check if selection is NOT inside the chat widget
        if (
          anchorNode &&
          !anchorNode.parentElement?.closest(".chat-widget-wrapper")
        ) {
          setSelectedText(text);
        }
      }
    };

    document.addEventListener("selectionchange", handleSelectionChange);
    return () => {
      document.removeEventListener("selectionchange", handleSelectionChange);
    };
  }, []);

  const handleSendMessage = async () => {
    if (!input.trim()) return;
    if (!user) {
      setMessages((prev) => [
        ...prev,
        { role: "assistant", content: "Please Sign In to chat with me!" },
      ]);
      return;
    }

    // Include selected text context in the message if available
    const contextNote = selectedText
      ? `\n\n[Context: "${selectedText.substring(0, 100)}${
          selectedText.length > 100 ? "..." : ""
        }"]`
      : "";
    const displayMessage = input + contextNote;

    const newMessages = [
      ...messages,
      { role: "user", content: displayMessage },
    ];
    setMessages(newMessages);
    setInput("");
    setLoading(true);

    try {
      const response = await fetch(chatUrls.chat(), {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Authorization: `Bearer ${localStorage.getItem("auth_token") || ""}`,
          },
        body: JSON.stringify({
          query: input,
          selected_text: selectedText, // Now passes actual selected text (FR-005)
          session_id: currentSessionId, // T170: Include session ID for stateful chat
          }),
        });

      // Clear selected text after sending
      setSelectedText("");
      const data = await response.json();

      // T170: Save session ID from response
      if (data.session_id && user) {
        setCurrentSessionId(data.session_id);
      }

      if (data.response) {
        // Use streaming typing effect instead of instant display
        setTypingMessage({ role: "assistant", content: data.response });
      } else {
        setMessages((prev) => [
          ...prev,
          { role: "assistant", content: "Sorry, I encountered an error." },
        ]);
      }
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        {
          role: "assistant",
          content: "Network error. Is the backend running?",
        },
      ]);
    } finally {
      setLoading(false);
    }
  };

  // T170: Session management functions
  const handleSelectSession = async (sessionId) => {
    if (!sessionId) {
      // New chat
      startNewChat();
      return;
    }

    // Load existing session
    setViewingSession(true);
    setCurrentSessionId(sessionId);
    setShowHistory(false);
  };

  const handleMessagesLoaded = (loadedMessages) => {
    // When viewing a session, replace current messages with loaded ones
    setMessages(loadedMessages);
    setViewingSession(false); // Return to chat mode after loading
  };

  const startNewChat = () => {
    setMessages([
      {
        role: "assistant",
        content:
          "Hello! I am your Physical AI assistant. How can I help you today? Tip: Select/highlight text on the page and ask me about it!",
      },
    ]);
    setCurrentSessionId(null);
    setShowHistory(false);
    setViewingSession(false);
  };

  // T178: Enhanced auth state checking - Verify token validity
  const checkAuthState = async () => {
    try {
      const token = localStorage.getItem("auth_token");
      if (!token) {
        setAuthChecked(true);
        return false;
      }

      // Verify token with backend
      const response = await fetch(
        `${API_BASE_URL}/api/auth/verify`,
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      if (response.status === 404) {
        // Endpoint not found - backend might not have this endpoint yet
        // Fall back to trusting the AuthProvider's user state
        console.warn(
          "/api/auth/verify endpoint not found, using AuthProvider state"
        );
        setAuthChecked(true);
        return !!user;
      }

      if (!response.ok) {
        // Token invalid, clear it
        localStorage.removeItem("auth_token");
        setAuthChecked(true);
        return false;
      }

      setAuthChecked(true);
      return true;
    } catch (error) {
      console.error("Auth check failed:", error);
      setAuthChecked(true);
      // On error, trust the AuthProvider state
      return !!user;
    }
  };

  // T178: Check auth state when widget opens
  useEffect(() => {
    if (isOpen && !authChecked) {
      checkAuthState();
    }
  }, [isOpen, authChecked]);

  // T179: Login prompt for anonymous users
  const showLoginPrompt = (feature) => {
    const loginUrl = "/login"; // Update with actual login page URL
    const message = `Please sign in to access ${feature}`;

    if (confirm(`${message}\n\nWould you like to go to the login page?`)) {
      window.location.href = loginUrl;
    }
  };

  const toggleHistory = () => {
    if (!user) {
      showLoginPrompt("chat history");
      return;
    }
    setShowHistory(!showHistory);
  };

  // T174: Session title update
  const updateSessionTitle = async (sessionId, newTitle) => {
    if (!sessionId || !newTitle.trim()) return;

    try {
      const token = localStorage.getItem("auth_token");
      const response = await fetch(
        chatUrls.updateTitle(sessionId),
        {
          method: "PATCH",
          headers: {
            Authorization: `Bearer ${token}`,
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ title: newTitle }),
        }
      );

      if (!response.ok) {
        throw new Error("Failed to update title");
      }

      return true;
    } catch (error) {
      console.error("Failed to update session title:", error);
      alert("Failed to update session title");
      return false;
    }
  };

  // T184: Error handler with retry capability
  const handleError = (error, operation, retryFn = null) => {
    console.error(`Error in ${operation}:`, error);
    setLastError({ operation, message: error.message });

    if (retryFn) {
      setRetryCallback(() => retryFn);
    }

    return false;
  };

  // T184: Retry last failed operation
  const retryLastOperation = () => {
    if (retryCallback) {
      setLastError(null);
      retryCallback();
      setRetryCallback(null);
    }
  };

  // T175: Delete session with T182/T183 enhancements
  const deleteSession = async (sessionId) => {
    if (!confirm("Delete this chat session? This cannot be undone.")) {
      return false;
    }

    try {
      // T182: Set loading state
      setOperationLoading(`delete-${sessionId}`);

      const token = localStorage.getItem("auth_token");
      const response = await fetch(
        chatUrls.deleteSession(sessionId),
        {
          method: "DELETE",
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      if (!response.ok) {
        throw new Error("Failed to delete session");
      }

      // T183: Optimistic UI - Reset immediately
      if (sessionId === currentSessionId) {
        startNewChat();
      }

      return true;
    } catch (error) {
      // T184: Handle error with retry
      return handleError(error, "delete session", () =>
        deleteSession(sessionId)
      );
    } finally {
      setOperationLoading(null);
    }
  };

  // T176: Clear all history
  const clearAllHistory = async () => {
    if (
      !confirm(
        "Clear ALL chat history? This will delete all your conversations and cannot be undone."
      )
    ) {
      return false;
    }

    try {
      const token = localStorage.getItem("auth_token");
      const response = await fetch("/api/chat/history", {
        method: "DELETE",
        headers: {
          Authorization: `Bearer ${token}`,
        },
      });

      if (!response.ok) {
        throw new Error("Failed to clear history");
      }

      const data = await response.json();
      alert(data.message || "Chat history cleared successfully");

      // Reset to new chat
      startNewChat();
      setShowHistory(false);

      return true;
    } catch (error) {
      console.error("Failed to clear history:", error);
      alert("Failed to clear chat history");
      return false;
    }
  };

  // T177: Export session
  const exportSession = async (sessionId, format = "text") => {
    try {
      const token = localStorage.getItem("auth_token");
      const response = await fetch(
        chatUrls.exportSession(sessionId, format),
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      if (!response.ok) {
        throw new Error("Failed to export session");
      }

      const data = await response.json();

      // Create and download file
      const content =
        format === "json"
          ? JSON.stringify(data.content, null, 2)
          : data.content;
      const blob = new Blob([content], {
        type: format === "json" ? "application/json" : "text/plain",
      });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `chat-session-${sessionId.substring(0, 8)}.${
        format === "json" ? "json" : "txt"
      }`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);

      return true;
    } catch (error) {
      console.error("Failed to export session:", error);
      alert("Failed to export chat session");
      return false;
    }
  };

  // T185: Keyboard shortcuts
  useEffect(() => {
    const handleKeyboardShortcuts = (e) => {
      // Ctrl+N or Cmd+N for new chat
      if ((e.ctrlKey || e.metaKey) && e.key === "n" && user && isOpen) {
        e.preventDefault();
        startNewChat();
      }
      // Ctrl+H or Cmd+H for history
      if ((e.ctrlKey || e.metaKey) && e.key === "h" && user && isOpen) {
        e.preventDefault();
        toggleHistory();
      }
    };

    window.addEventListener("keydown", handleKeyboardShortcuts);
    return () => window.removeEventListener("keydown", handleKeyboardShortcuts);
  }, [user, isOpen, showHistory]);

  return (
    <div className="chat-widget-wrapper">
      {!isOpen && (
        <button
          className="chat-toggle-btn pulse-animation"
          onClick={() => setIsOpen(true)}
          aria-label="Open AI Assistant"
        >
          <ChatIcon />
        </button>
      )}

      {isOpen && (
        <div className="chat-window" role="dialog" aria-label="AI Assistant Chat" aria-modal="false">
          <div className="chat-header">
            <div className="header-title">
              <span className="chat-icon-wrapper"><ChatIcon /></span>
              <span>AI Assistant</span>
            </div>
            <div className="header-actions">
              {/* T170: New Chat button */}
              {user && !viewingSession && (
                <button
                  className="icon-btn"
                  onClick={startNewChat}
                  title="New chat"
                  aria-label="Start new chat"
                >
                  <NewChatIcon />
                </button>
              )}
              {/* T170: History button */}
              {user && !viewingSession && (
                <button
                  className="icon-btn"
                  onClick={toggleHistory}
                  title="Chat history"
                  aria-label="View chat history"
                >
                  <HistoryIcon />
                </button>
              )}
              <button
                className="close-btn"
                onClick={() => setIsOpen(false)}
                aria-label="Close chat"
              >
                <CloseIcon />
              </button>
            </div>
          </div>

          {/* T170: Show ChatHistory or ChatSession or regular chat */}
          {showHistory ? (
            <ChatHistory
              onSelectSession={handleSelectSession}
              onNewChat={startNewChat}
              currentSessionId={currentSessionId}
              onClose={() => setShowHistory(false)}
              onDeleteSession={deleteSession}
              onExportSession={exportSession}
              onClearAll={clearAllHistory}
              user={user} // T194: Pass user prop to ChatHistory
              authLoading={authLoading} // T195: Pass auth loading state
            />
          ) : viewingSession ? (
            <ChatSession
              sessionId={currentSessionId}
              onBack={() => setViewingSession(false)}
              onMessagesLoaded={handleMessagesLoaded}
            />
          ) : (
            <>
              <div className="chat-messages" role="log" aria-live="polite" aria-atomic="false" aria-busy={loading}>
                {messages.map((msg, idx) => (
                  <div key={idx} className={`message-row ${msg.role}`}>
                    {msg.role === "assistant" && (
                      <div className="bot-avatar">
                        <span className="chat-icon-wrapper"><ChatIcon /></span>
                      </div>
                    )}
                    <div className={`message-bubble ${msg.role}`}>
                      {msg.content}
                    </div>
                  </div>
                ))}
                {/* Streaming typing effect message */}
                {typingMessage && streamedContent && (
                  <div className="message-row assistant">
                    <div className="bot-avatar">
                      <span className="chat-icon-wrapper"><ChatIcon /></span>
                    </div>
                    <div className="message-bubble assistant streaming">
                      {streamedContent}
                      <span className="typing-cursor"></span>
                    </div>
                  </div>
                )}
                {loading && !typingMessage && (
                  <div className="message-row assistant">
                    <div className="bot-avatar">
                      <span className="chat-icon-wrapper"><ChatIcon /></span>
                    </div>
                    <div className="message-bubble assistant typing">
                      <div className="typing-dot"></div>
                      <div className="typing-dot"></div>
                      <div className="typing-dot"></div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>

              {/* Selected text indicator (FR-005) */}
              {selectedText && (
                <div className="selected-text-indicator">
                  <span className="indicator-icon">📝</span>
                  <span className="indicator-text">
                    Context: "{selectedText.substring(0, 50)}
                    {selectedText.length > 50 ? "..." : ""}"
                  </span>
                  <button
                    className="clear-selection-btn"
                    onClick={() => setSelectedText("")}
                    title="Clear selection"
                    aria-label="Clear selected text context"
                  >
                    ✕
                  </button>
                </div>
              )}

              {/* T170: Only show input area when in chat mode (not history/session view) */}
              <div className="chat-input-area">
                <input
                  type="text"
                  value={input}
                  onChange={(e) => setInput(e.target.value)}
                  onKeyPress={(e) => e.key === "Enter" && handleSendMessage()}
                  placeholder={
                    selectedText
                      ? "Ask about selected text..."
                      : "Ask a question..."
                  }
                  disabled={loading}
                  aria-label="Type your message"
                />
                <button
                  onClick={handleSendMessage}
                  disabled={loading || !input.trim()}
                  className="send-btn"
                  aria-label="Send message"
                >
                  <SendIcon />
                </button>
              </div>
            </>
          )}
        </div>
      )}

      {/* 🟢 STYLES with MOBILE RESPONSIVENESS */}
      <style>{`
        .chat-widget-wrapper { position: fixed; bottom: 25px; right: 25px; z-index: 9999; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        
        .chat-toggle-btn { width: 65px; height: 65px; border-radius: 50%; background: transparent; border: none; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s; padding: 0; }
        .chat-toggle-btn:hover { transform: scale(1.05); }
        .chat-toggle-btn:focus-visible { outline: 2px solid var(--module-primary, #2ecc71); outline-offset: 2px; }
        .pulse-animation { animation: pulse 2s infinite; }
        
        .chat-window { width: 380px; height: 600px; max-height: 80vh; background-color: #1e2a38; border-radius: 20px; box-shadow: 0 20px 50px rgba(0, 0, 0, 0.5); display: flex; flex-direction: column; border: 1px solid #2c3e50; overflow: hidden; animation: slideUp 0.3s cubic-bezier(0.16, 1, 0.3, 1); }
        
        /* 📱 MOBILE RESPONSIVENESS */
        @media (max-width: 480px) {
          .chat-widget-wrapper { bottom: 15px; right: 15px; }
          .chat-window {
            width: calc(100vw - 30px); /* Full width minus margin */
            height: 70vh; /* Taller on mobile */
            position: fixed;
            bottom: 80px; /* Above the toggle button */
            right: 15px;
            left: 15px; /* Center it */
          }
        }

        .chat-header { padding: 18px 20px; background: linear-gradient(90deg, #2ecc71, #27ae60); color: #1e2a38; display: flex; justify-content: space-between; align-items: center; }
        .header-title { display: flex; align-items: center; gap: 10px; font-weight: 800; font-size: 16px; }
        .header-title .chat-icon-wrapper { transform: scale(0.7); }
        .header-actions { display: flex; gap: 8px; align-items: center; }
        .icon-btn { background: rgba(255,255,255,0.2); border: none; border-radius: 50%; width: 32px; height: 32px; display: flex; align-items: center; justify-content: center; cursor: pointer; color: #1e2a38; transition: background-color 0.2s; }
        .icon-btn:hover { background: rgba(255,255,255,0.3); }
        .icon-btn:focus-visible { outline: 2px solid #1e2a38; outline-offset: 2px; }
        .close-btn { background: rgba(255,255,255,0.2); border: none; border-radius: 50%; width: 32px; height: 32px; display: flex; align-items: center; justify-content: center; cursor: pointer; color: #1e2a38; }
        .close-btn:focus-visible { outline: 2px solid #1e2a38; outline-offset: 2px; }

        .chat-messages { flex: 1; padding: 20px; overflow-y: auto; display: flex; flex-direction: column; gap: 15px; background-color: #151e29; }
        .message-row { display: flex; gap: 10px; align-items: flex-end; animation: fadeIn 0.3s ease; }
        .message-row.user { justify-content: flex-end; }
        .bot-avatar { width: 30px; height: 30px; background: #2c3e50; border-radius: 50%; display: flex; align-items: center; justify-content: center; flex-shrink: 0; padding: 0; }
        .bot-avatar .chat-icon-wrapper { transform: scale(0.6); }
        
        .message-bubble { padding: 12px 16px; border-radius: 18px; max-width: 80%; line-height: 1.5; font-size: 14px; position: relative; box-shadow: 0 1px 2px rgba(0,0,0,0.2); }
        .message-bubble.assistant { background-color: #2c3e50; color: #e2e8f0; border-bottom-left-radius: 4px; }
        .message-bubble.user { background-color: #2ecc71; color: #1e2a38; border-bottom-right-radius: 4px; font-weight: 500; }
        
        .message-bubble.typing { padding: 15px 20px; display: flex; align-items: center; gap: 4px; min-height: 40px; }
        .typing-dot { width: 8px; height: 8px; background-color: #94a3b8; border-radius: 50%; animation: bounce 1.4s infinite ease-in-out both; }
        .typing-dot:nth-child(1) { animation-delay: -0.32s; }
        .typing-dot:nth-child(2) { animation-delay: -0.16s; }

        /* Streaming typing effect */
        .message-bubble.streaming { display: flex; align-items: flex-start; gap: 2px; }
        .typing-cursor { display: inline-block; width: 2px; height: 16px; background-color: #2ecc71; animation: blink 0.8s infinite; margin-left: 2px; flex-shrink: 0; }

        @keyframes blink { 0%, 49% { opacity: 1; } 50%, 100% { opacity: 0; } }

        /* Selected text indicator (FR-005) */
        .selected-text-indicator { padding: 8px 15px; background-color: #2c3e50; border-top: 1px solid #3d5266; display: flex; align-items: center; gap: 8px; font-size: 12px; color: #94a3b8; }
        .indicator-icon { font-size: 14px; }
        .indicator-text { flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: #a8d5ff; }
        .clear-selection-btn { background: none; border: none; color: #94a3b8; cursor: pointer; padding: 2px 6px; border-radius: 4px; font-size: 12px; }
        .clear-selection-btn:hover { background-color: #3d5266; color: white; }

        .chat-input-area { padding: 20px; background-color: #1e2a38; border-top: 1px solid #2c3e50; display: flex; gap: 10px; align-items: center; }
        .chat-input-area input { flex: 1; padding: 12px 15px; border-radius: 25px; border: 1px solid #2c3e50; background-color: #151e29; color: white; font-size: 14px; }
        .chat-input-area input:focus { outline: none; border-color: #2ecc71; box-shadow: 0 0 0 2px rgba(46, 204, 113, 0.2); }
        .send-btn { width: 45px; height: 45px; background-color: #2ecc71; color: #1e2a38; border: none; border-radius: 50%; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s; }
        .send-btn:hover:not(:disabled) { transform: scale(1.1); }
        .send-btn:focus-visible { outline: 2px solid white; outline-offset: 2px; }
        .send-btn:disabled { background-color: #2c3e50; color: #94a3b8; cursor: not-allowed; }

        @keyframes bounce { 0%, 80%, 100% { transform: scale(0); } 40% { transform: scale(1); } }
        @keyframes slideUp { from { opacity: 0; transform: translateY(30px) scale(0.95); } to { opacity: 1; transform: translateY(0) scale(1); } }
        @keyframes fadeIn { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }

        /* Reduced motion support */
        @media (prefers-reduced-motion: reduce) {
          .chat-toggle-btn, .send-btn, .icon-btn, .close-btn { transition: none; }
          .message-row, .chat-window { animation: none; }
          .typing-dot { animation: none; }
          .typing-cursor { animation: none; }
          .pulse-animation { animation: none; }
        }
      `}</style>
    </div>
  );
};

const ChatWidget = () => {
  return (
    <BrowserOnly fallback={null}>{() => <ChatWidgetContent />}</BrowserOnly>
  );
};

export default ChatWidget;
