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
//
// The chat opens empty on purpose: it never ships a pre-seeded greeting
// that looks like a conversation that never happened. An earlier version
// seeded "Hello! I am your Physical AI assistant..." on every fresh load.

const ChatIcon = EnhancedChatIcon;

const SESSION_KEY = "physical-ai-chat-session";
const MESSAGES_KEY = "physical-ai-chat-messages";

// The old pre-seeded greeting, dropped when loading stored history.
const OLD_GREETING =
  "Hello! I am your Physical AI assistant. Ask me anything about the book. Tip: highlight text on the page and ask me about it.";

function newSessionId() {
  if (typeof crypto !== "undefined" && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  return `session-${Date.now()}-${Math.floor(Math.random() * 100000)}`;
}

function loadStoredMessages() {
  if (typeof window === "undefined") return [];
  try {
    const raw = localStorage.getItem(MESSAGES_KEY);
    const parsed = raw ? JSON.parse(raw) : null;
    if (Array.isArray(parsed)) {
      return parsed.filter((m) => m && m.content !== OLD_GREETING);
    }
  } catch {
    // Corrupt storage — start fresh.
  }
  return [];
}

function loadStoredSessionId() {
  if (typeof window === "undefined") return null;
  try {
    return localStorage.getItem(SESSION_KEY) || null;
  } catch {
    return null;
  }
}

/**
 * Small safe markdown renderer for assistant answers. HTML is escaped
 * first, then a subset is rendered: bold, italic, inline code, fenced
 * code blocks, unordered/ordered lists, paragraphs. No dependency.
 */
function escapeHtml(text) {
  return text
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function renderMarkdown(text) {
  const blocks = [];
  let html = escapeHtml(text);
  html = html.replace(/```(\w*)\n?([\s\S]*?)```/g, (_m, _lang, code) => {
    blocks.push(`<pre class="md-code"><code>${code.replace(/\n$/, "")}</code></pre>`);
    return `\u0000${blocks.length - 1}\u0000`;
  });
  html = html.replace(/`([^`\n]+)`/g, '<code class="md-inline-code">$1</code>');
  html = html.replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");
  html = html.replace(/(^|[^*])\*([^*\n]+)\*/g, "$1<em>$2</em>");

  const lines = html.split("\n");
  let out = "";
  let inList = null;
  const closeList = () => {
    if (inList) {
      out += `</${inList}>`;
      inList = null;
    }
  };
  for (const line of lines) {
    const placeholder = line.trim().match(/^\u0000(\d+)\u0000$/);
    if (placeholder) {
      closeList();
      out += blocks[Number(placeholder[1])];
      continue;
    }
    const ul = line.match(/^\s*[-*]\s+(.*)$/);
    const ol = line.match(/^\s*\d+[.)]\s+(.*)$/);
    if (ul) {
      if (inList !== "ul") {
        closeList();
        out += '<ul class="md-list">';
        inList = "ul";
      }
      out += `<li>${ul[1]}</li>`;
    } else if (ol) {
      if (inList !== "ol") {
        closeList();
        out += '<ol class="md-list">';
        inList = "ol";
      }
      out += `<li>${ol[1]}</li>`;
    } else if (line.trim() === "") {
      closeList();
    } else {
      closeList();
      out += `<p class="md-p">${line}</p>`;
    }
  }
  closeList();
  return out;
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

const SmallCloseIcon = () => (
  <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
    <line x1="18" y1="6" x2="6" y2="18"></line>
    <line x1="6" y1="6" x2="6" y2="18"></line>
  </svg>
);

const QuoteIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M3 21c3 0 7-1 7-8V5c0-1.25-.756-2.017-2-2H4c-1.25 0-2 .75-2 1.972V11c0 1.25.75 2 2 2 1 0 1 0 1 1v1c0 1-1 2-2 2s-1 .008-1 1.031V20c0 1 0 1 1 1z"></path>
    <path d="M15 21c3 0 7-1 7-8V5c0-1.25-.757-2.017-2-2h-4c-1.25 0-2 .75-2 1.972V11c0 1.25.75 2 2 2h.75c0 2.25.25 4-2.75 4v3c0 1 0 1 1 1z"></path>
  </svg>
);

const NewChatIcon = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <line x1="12" y1="5" x2="12" y2="19"></line>
    <line x1="5" y1="12" x2="19" y2="12"></line>
  </svg>
);

// Real questions a reader can ask the book — they fill the input, they
// are not answers and not a fake conversation.
const SUGGESTIONS = [
  "What is ROS 2?",
  "What is the reality gap?",
  "How do I start Week 1?",
];

const ChatWidgetContent = () => {
  const { user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
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

  // Persist messages whenever they change.
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
      localStorage.setItem(MESSAGES_KEY, JSON.stringify([]));
    } catch {
      // Storage unavailable — ignore.
    }
    setMessages([]);
    setStreamedContent("");
  };

  const handleSendMessage = async (overrideText) => {
    const text = (overrideText ?? input).trim();
    if (!text || loading) return;
    if (!user) {
      setMessages((prev) => [
        ...prev,
        { role: "assistant", content: "Please sign in to chat with me!" },
      ]);
      setInput("");
      return;
    }

    const question = text;
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
            {messages.length === 0 && !loading && !streamedContent && (
              <div className="chat-empty-state">
                <p className="chat-empty-title">Ask the book anything</p>
                <p className="chat-empty-text">
                  Answers come from the chapters. Sign in to start chatting.
                </p>
                <div className="chat-suggestions">
                  {SUGGESTIONS.map((suggestion) => (
                    <button
                      key={suggestion}
                      className="chat-suggestion"
                      onClick={() => handleSendMessage(suggestion)}
                    >
                      {suggestion}
                    </button>
                  ))}
                </div>
              </div>
            )}
            {messages.map((msg, idx) => (
              <div key={idx} className={`message-row ${msg.role}`}>
                {msg.role === "assistant" && (
                  <div className="bot-avatar">
                    <span className="chat-icon-wrapper"><ChatIcon /></span>
                  </div>
                )}
                {msg.role === "assistant" ? (
                  <div
                    className={`message-bubble ${msg.role}`}
                    dangerouslySetInnerHTML={{ __html: renderMarkdown(msg.content) }}
                  />
                ) : (
                  <div className={`message-bubble ${msg.role}`}>{msg.content}</div>
                )}
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
              <span className="indicator-icon"><QuoteIcon /></span>
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
                <SmallCloseIcon />
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
              onClick={() => handleSendMessage()}
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
        .chat-widget-wrapper { position: fixed; bottom: 25px; right: 25px; z-index: 9999; font-family: var(--font-body); }

        .chat-toggle-btn { width: 65px; height: 65px; border-radius: 50%; background: transparent; border: none; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s ease; padding: 0; }
        .chat-toggle-btn:hover { transform: scale(1.05); }
        .chat-toggle-btn:focus-visible { outline: 2px solid var(--module-primary); outline-offset: 2px; }
        .pulse-animation { animation: pulse 2s infinite; }

        .chat-window { width: 380px; height: 600px; max-height: 80vh; background-color: var(--chat-bg-primary); border-radius: 20px; box-shadow: var(--shadow-2xl); display: flex; flex-direction: column; border: 1px solid var(--chat-border); overflow: hidden; animation: slideUp 0.3s cubic-bezier(0.16, 1, 0.3, 1); }

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

        .chat-header { padding: 18px 20px; background: var(--chat-bubble-user); color: var(--chat-bubble-user-text); display: flex; justify-content: space-between; align-items: center; }
        .header-title { display: flex; align-items: center; gap: 10px; font-weight: 800; font-size: 16px; }
        .header-title .chat-icon-wrapper { transform: scale(0.7); }
        .header-actions { display: flex; gap: 8px; align-items: center; }
        .icon-btn { background: var(--color-overlay-light); border: none; border-radius: 50%; width: 32px; height: 32px; display: flex; align-items: center; justify-content: center; cursor: pointer; color: var(--chat-bubble-user-text); transition: background-color 0.2s ease; }
        .icon-btn:hover { background: var(--color-overlay-light); }
        .icon-btn:focus-visible { outline: 2px solid var(--chat-bubble-user-text); outline-offset: 2px; }
        .close-btn { background: var(--color-overlay-light); border: none; border-radius: 50%; width: 32px; height: 32px; display: flex; align-items: center; justify-content: center; cursor: pointer; color: var(--chat-bubble-user-text); }
        .close-btn:focus-visible { outline: 2px solid var(--chat-bubble-user-text); outline-offset: 2px; }

        .chat-messages { flex: 1; padding: 20px; overflow-y: auto; display: flex; flex-direction: column; gap: 15px; background-color: var(--chat-bg-surface); }
        .chat-empty-state { padding: 32px 8px; text-align: center; }
        .chat-empty-title { font-size: 17px; font-weight: 700; color: var(--color-text-primary); margin: 0 0 8px; }
        .chat-empty-text { font-size: 14px; line-height: 1.5; color: var(--color-text-secondary); margin: 0 0 20px; }
        .chat-suggestions { display: flex; flex-direction: column; gap: 8px; }
        .chat-suggestion { padding: 10px 14px; border-radius: 20px; border: 1px solid var(--chat-border); background-color: var(--chat-bg-primary); color: var(--color-text-primary); font-size: 13px; cursor: pointer; text-align: left; transition: border-color 0.2s ease, background-color 0.2s ease; }
        .chat-suggestion:hover { border-color: var(--module-primary); background-color: var(--module-primary-bg); }

        .message-row { display: flex; gap: 10px; align-items: flex-end; animation: fadeIn 0.3s ease; }
        .message-row.user { justify-content: flex-end; }
        .bot-avatar { width: 30px; height: 30px; background: var(--chat-bubble-bot); border-radius: 50%; display: flex; align-items: center; justify-content: center; flex-shrink: 0; padding: 0; }
        .bot-avatar .chat-icon-wrapper { transform: scale(0.6); }

        .message-bubble { padding: 12px 16px; border-radius: 18px; max-width: 80%; line-height: 1.5; font-size: 14px; position: relative; box-shadow: var(--shadow-sm); word-break: break-word; }
        .message-bubble.assistant { background-color: var(--chat-bubble-bot); color: var(--chat-bubble-bot-text); border-bottom-left-radius: 4px; }
        .message-bubble.user { background-color: var(--chat-bubble-user); color: var(--chat-bubble-user-text); border-bottom-right-radius: 4px; font-weight: 500; }
        .message-bubble .md-p { margin: 0 0 8px; }
        .message-bubble .md-p:last-child { margin-bottom: 0; }
        .message-bubble .md-list { margin: 0 0 8px; padding-left: 18px; }
        .message-bubble .md-list:last-child { margin-bottom: 0; }
        .message-bubble .md-list li { margin-bottom: 2px; }
        .message-bubble .md-inline-code { font-family: var(--font-code); font-size: 0.85em; background-color: var(--color-bg-tertiary); padding: 1px 5px; border-radius: 4px; }
        .message-bubble .md-code { margin: 0 0 8px; padding: 10px 12px; border-radius: 8px; background-color: var(--color-bg-tertiary); overflow-x: auto; font-size: 13px; line-height: 1.5; }
        .message-bubble .md-code:last-child { margin-bottom: 0; }
        .message-bubble .md-code code { font-family: var(--font-code); }

        .message-bubble.typing { padding: 15px 20px; display: flex; align-items: center; gap: 4px; min-height: 40px; }
        .typing-dot { width: 8px; height: 8px; background-color: var(--color-text-tertiary); border-radius: 50%; animation: bounce 1.4s infinite ease-in-out both; }
        .typing-dot:nth-child(1) { animation-delay: -0.32s; }
        .typing-dot:nth-child(2) { animation-delay: -0.16s; }

        .message-bubble.streaming { display: flex; align-items: flex-start; gap: 2px; white-space: pre-wrap; }
        .typing-cursor { display: inline-block; width: 2px; height: 16px; background-color: var(--module-primary); animation: blink 0.8s infinite; margin-left: 2px; flex-shrink: 0; }

        @keyframes blink { 0%, 49% { opacity: 1; } 50%, 100% { opacity: 0; } }

        .selected-text-indicator { padding: 8px 15px; background-color: var(--chat-bubble-bot); border-top: 1px solid var(--chat-border); display: flex; align-items: center; gap: 8px; font-size: 12px; color: var(--color-text-secondary); }
        .indicator-icon { display: flex; align-items: center; }
        .indicator-text { flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: var(--color-text-primary); }
        .clear-selection-btn { background: none; border: none; color: var(--color-text-secondary); cursor: pointer; padding: 2px 6px; border-radius: 4px; display: flex; align-items: center; }
        .clear-selection-btn:hover { background-color: var(--color-bg-tertiary); color: var(--color-text-primary); }

        .chat-input-area { padding: 20px; background-color: var(--chat-bg-primary); border-top: 1px solid var(--chat-border); display: flex; gap: 10px; align-items: center; }
        .chat-input-area input { flex: 1; height: 45px; padding: 0 15px; border-radius: 25px; border: 1px solid var(--chat-border); background-color: var(--chat-bg-surface); color: var(--color-text-primary); font-size: 14px; min-width: 0; }
        .chat-input-area input:focus { outline: none; border-color: var(--module-primary); box-shadow: 0 0 0 2px var(--color-focus-ring); }
        .send-btn { width: 45px; height: 45px; background-color: var(--chat-bubble-user); color: var(--chat-bubble-user-text); border: none; border-radius: 50%; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s ease; flex-shrink: 0; }
        .send-btn:hover:not(:disabled) { transform: scale(1.1); }
        .send-btn:focus-visible { outline: 2px solid var(--module-primary); outline-offset: 2px; }
        .send-btn:disabled { background-color: var(--chat-bubble-bot); color: var(--color-text-tertiary); cursor: not-allowed; }

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
