import React, { useState, useEffect, useRef } from "react";
import { useAuth } from "../Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";
import EnhancedChatIcon from "./ChatIcon.enhanced";
import "./ChatIcon.enhanced.css";
import { chatUrls, readSseStream } from "../../config/api";

// The backend stores chat history by session id but has no endpoint to
// list or fetch past sessions, so this widget keeps one conversation per
// browser: the session id and the messages live in localStorage and
// survive page reloads. "New chat" starts a fresh conversation.

const ChatIcon = EnhancedChatIcon;

const SESSION_KEY = "physical-ai-chat-session";
const MESSAGES_KEY = "physical-ai-chat-messages";

const GREETING = {
  role: "assistant",
  content:
    "Hello! I am your Physical AI assistant. Ask me anything about the book. Tip: highlight text on the page and ask me about it.",
};

function newSessionId() {
  if (typeof crypto !== "undefined" && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  return `session-${Date.now()}-${Math.floor(Math.random() * 100000)}`;
}

function loadStoredMessages() {
  if (typeof window === "undefined") return [GREETING];
  try {
    const raw = localStorage.getItem(MESSAGES_KEY);
    const parsed = raw ? JSON.parse(raw) : null;
    if (Array.isArray(parsed) && parsed.length > 0) return parsed;
  } catch {
    // Corrupt storage — start fresh.
  }
  return [GREETING];
}

function loadStoredSessionId() {
  if (typeof window === "undefined") return null;
  try {
    return localStorage.getItem(SESSION_KEY) || null;
  } catch {
    return null;
  }
}

const SendIcon = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <line x1="22" y1="2" x2="11" y2="13"></line>
    <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
  </svg>
);

const CloseIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <line x1="18" y1="6" x2="6" y2="18"></line>
    <line x1="6" y1="6" x2="6" y2="18"></line>
  </svg>
);

const NewChatIcon = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <line x1="12" y1="5" x2="12" y2="19"></line>
    <line x1="5" y1="12" x2="19" y2="12"></line>
  </svg>
);

const ChatWidgetContent = () => {
  const { user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([GREETING]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const [streamedContent, setStreamedContent] = useState("");
  const [selectedText, setSelectedText] = useState("");
  const messagesEndRef = useRef(null);
  const sessionIdRef = useRef(null);

  // Load the stored conversation on mount (client only).
  useEffect(() => {
    setMessages(loadStoredMessages());
    let id = loadStoredSessionId();
    if (!id) {
      id = newSessionId();
      try {
        localStorage.setItem(SESSION_KEY, id);
      } catch {
        // Storage unavailable — the session still works for this visit.
      }
    }
    sessionIdRef.current = id;
  }, []);

  // Persist messages whenever they change (skip the initial greeting-only state).
  useEffect(() => {
    if (typeof window === "undefined") return;
    try {
      localStorage.setItem(MESSAGES_KEY, JSON.stringify(messages));
    } catch {
      // Storage unavailable — ignore.
    }
  }, [messages]);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages, isOpen, loading, streamedContent]);

  // Capture text selected on the page (not inside the chat widget).
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || "";
      if (text && text.length > 0 && text.length < 5000) {
        const anchorNode = selection?.anchorNode;
        if (anchorNode && !anchorNode.parentElement?.closest(".chat-widget-wrapper")) {
          setSelectedText(text);
        }
      }
    };
    document.addEventListener("selectionchange", handleSelectionChange);
    return () => document.removeEventListener("selectionchange", handleSelectionChange);
  }, []);

  const startNewChat = () => {
    const id = newSessionId();
    sessionIdRef.current = id;
    try {
      localStorage.setItem(SESSION_KEY, id);
      localStorage.setItem(MESSAGES_KEY, JSON.stringify([GREETING]));
    } catch {
      // Storage unavailable — ignore.
    }
    setMessages([GREETING]);
    setStreamedContent("");
  };

  const handleSendMessage = async () => {
    if (!input.trim() || loading) return;
    if (!user) {
      setMessages((prev) => [
        ...prev,
        { role: "assistant", content: "Please sign in to chat with me!" },
      ]);
      setInput("");
      return;
    }

    const question = input.trim();
    const contextText = selectedText;
    setMessages((prev) => [...prev, { role: "user", content: question }]);
    setInput("");
    setSelectedText("");
    setLoading(true);
    setStreamedContent("");

    try {
      const useSelected = Boolean(contextText);
      const response = await fetch(useSelected ? chatUrls.chatSelected() : chatUrls.chat(), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(
          useSelected
            ? { message: question, selected_text: contextText, session_id: sessionIdRef.current }
            : { message: question, session_id: sessionIdRef.current }
        ),
      });

      const fullText = await readSseStream(response, (soFar) => setStreamedContent(soFar));
      setStreamedContent("");
      if (fullText.trim()) {
        setMessages((prev) => [...prev, { role: "assistant", content: fullText.trim() }]);
      } else {
        setMessages((prev) => [
          ...prev,
          { role: "assistant", content: "I could not find an answer in the book for that. Try asking about ROS 2, Gazebo, NVIDIA Isaac, or humanoid robots." },
        ]);
      }
    } catch (error) {
      console.error("Chat request failed:", error);
      setStreamedContent("");
      setMessages((prev) => [
        ...prev,
        {
          role: "assistant",
          content:
            "Sorry, I could not reach the assistant just now. Please check your connection and try again.",
        },
      ]);
    } finally {
      setLoading(false);
    }
  };

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
              {user && (
                <button
                  className="icon-btn"
                  onClick={startNewChat}
                  title="New chat"
                  aria-label="Start new chat"
                >
                  <NewChatIcon />
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

          <div className="chat-messages" role="log" aria-live="polite" aria-atomic="false" aria-busy={loading}>
            {messages.map((msg, idx) => (
              <div key={idx} className={`message-row ${msg.role}`}>
                {msg.role === "assistant" && (
                  <div className="bot-avatar">
                    <span className="chat-icon-wrapper"><ChatIcon /></span>
                  </div>
                )}
                <div className={`message-bubble ${msg.role}`}>{msg.content}</div>
              </div>
            ))}
            {streamedContent && (
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
            {loading && !streamedContent && (
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

          <div className="chat-input-area">
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === "Enter" && handleSendMessage()}
              placeholder={selectedText ? "Ask about selected text..." : "Ask a question..."}
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
        </div>
      )}

      <style>{`
        .chat-widget-wrapper { position: fixed; bottom: 25px; right: 25px; z-index: 9999; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }

        .chat-toggle-btn { width: 65px; height: 65px; border-radius: 50%; background: transparent; border: none; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s; padding: 0; }
        .chat-toggle-btn:hover { transform: scale(1.05); }
        .chat-toggle-btn:focus-visible { outline: 2px solid var(--module-primary, #2ecc71); outline-offset: 2px; }
        .pulse-animation { animation: pulse 2s infinite; }

        .chat-window { width: 380px; height: 600px; max-height: 80vh; background-color: #1e2a38; border-radius: 20px; box-shadow: 0 20px 50px rgba(0, 0, 0, 0.5); display: flex; flex-direction: column; border: 1px solid #2c3e50; overflow: hidden; animation: slideUp 0.3s cubic-bezier(0.16, 1, 0.3, 1); }

        @media (max-width: 480px) {
          .chat-widget-wrapper { bottom: 15px; right: 15px; }
          .chat-window {
            width: calc(100vw - 30px);
            height: 70vh;
            position: fixed;
            bottom: 80px;
            right: 15px;
            left: 15px;
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

        .message-bubble { padding: 12px 16px; border-radius: 18px; max-width: 80%; line-height: 1.5; font-size: 14px; position: relative; box-shadow: 0 1px 2px rgba(0,0,0,0.2); white-space: pre-wrap; word-break: break-word; }
        .message-bubble.assistant { background-color: #2c3e50; color: #e2e8f0; border-bottom-left-radius: 4px; }
        .message-bubble.user { background-color: #2ecc71; color: #1e2a38; border-bottom-right-radius: 4px; font-weight: 500; }

        .message-bubble.typing { padding: 15px 20px; display: flex; align-items: center; gap: 4px; min-height: 40px; }
        .typing-dot { width: 8px; height: 8px; background-color: #94a3b8; border-radius: 50%; animation: bounce 1.4s infinite ease-in-out both; }
        .typing-dot:nth-child(1) { animation-delay: -0.32s; }
        .typing-dot:nth-child(2) { animation-delay: -0.16s; }

        .message-bubble.streaming { display: flex; align-items: flex-start; gap: 2px; }
        .typing-cursor { display: inline-block; width: 2px; height: 16px; background-color: #2ecc71; animation: blink 0.8s infinite; margin-left: 2px; flex-shrink: 0; }

        @keyframes blink { 0%, 49% { opacity: 1; } 50%, 100% { opacity: 0; } }

        .selected-text-indicator { padding: 8px 15px; background-color: #2c3e50; border-top: 1px solid #3d5266; display: flex; align-items: center; gap: 8px; font-size: 12px; color: #94a3b8; }
        .indicator-icon { font-size: 14px; }
        .indicator-text { flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: #a8d5ff; }
        .clear-selection-btn { background: none; border: none; color: #94a3b8; cursor: pointer; padding: 2px 6px; border-radius: 4px; font-size: 12px; }
        .clear-selection-btn:hover { background-color: #3d5266; color: white; }

        .chat-input-area { padding: 20px; background-color: #1e2a38; border-top: 1px solid #2c3e50; display: flex; gap: 10px; align-items: center; }
        .chat-input-area input { flex: 1; padding: 12px 15px; border-radius: 25px; border: 1px solid #2c3e50; background-color: #151e29; color: white; font-size: 14px; min-width: 0; }
        .chat-input-area input:focus { outline: none; border-color: #2ecc71; box-shadow: 0 0 0 2px rgba(46, 204, 113, 0.2); }
        .send-btn { width: 45px; height: 45px; background-color: #2ecc71; color: #1e2a38; border: none; border-radius: 50%; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s; flex-shrink: 0; }
        .send-btn:hover:not(:disabled) { transform: scale(1.1); }
        .send-btn:focus-visible { outline: 2px solid white; outline-offset: 2px; }
        .send-btn:disabled { background-color: #2c3e50; color: #94a3b8; cursor: not-allowed; }

        @keyframes bounce { 0%, 80%, 100% { transform: scale(0); } 40% { transform: scale(1); } }
        @keyframes slideUp { from { opacity: 0; transform: translateY(30px) scale(0.95); } to { opacity: 1; transform: translateY(0) scale(1); } }
        @keyframes fadeIn { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }

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
