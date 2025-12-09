import React, { useState, useEffect, useRef } from "react";
import { useAuth } from "../Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";

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
const RobotIcon = () => (
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
    <rect x="3" y="11" width="18" height="10" rx="2"></rect>
    <circle cx="12" cy="5" r="2"></circle>
    <path d="M12 7v4"></path>
    <line x1="8" y1="16" x2="8" y2="16"></line>
    <line x1="16" y1="16" x2="16" y2="16"></line>
  </svg>
);

const ChatWidgetContent = () => {
  const { user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: "assistant",
      content:
        "Hello! I am your Physical AI assistant. How can I help you today?",
    },
  ]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };
  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen, loading]);

  const handleSendMessage = async () => {
    if (!input.trim()) return;
    if (!user) {
      setMessages((prev) => [
        ...prev,
        { role: "assistant", content: "Please Sign In to chat with me!" },
      ]);
      return;
    }

    const newMessages = [...messages, { role: "user", content: input }];
    setMessages(newMessages);
    setInput("");
    setLoading(true);

    try {
      const response = await fetch(
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/chat",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${localStorage.getItem("auth_token") || ""}`,
          },
          body: JSON.stringify({ query: input, selected_text: "" }),
        }
      );
      const data = await response.json();
      if (data.response) {
        setMessages((prev) => [
          ...prev,
          { role: "assistant", content: data.response },
        ]);
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

  return (
    <div className="chat-widget-wrapper">
      {!isOpen && (
        <button
          className="chat-toggle-btn pulse-animation"
          onClick={() => setIsOpen(true)}
        >
          <span style={{ fontSize: "24px" }}>🤖</span>
        </button>
      )}

      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <div className="header-title">
              <RobotIcon />
              <span>AI Assistant</span>
            </div>
            <button className="close-btn" onClick={() => setIsOpen(false)}>
              <CloseIcon />
            </button>
          </div>

          <div className="chat-messages">
            {messages.map((msg, idx) => (
              <div key={idx} className={`message-row ${msg.role}`}>
                {msg.role === "assistant" && (
                  <div className="bot-avatar">
                    <RobotIcon />
                  </div>
                )}
                <div className={`message-bubble ${msg.role}`}>
                  {msg.content}
                </div>
              </div>
            ))}
            {loading && (
              <div className="message-row assistant">
                <div className="bot-avatar">
                  <RobotIcon />
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

          <div className="chat-input-area">
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === "Enter" && handleSendMessage()}
              placeholder="Ask a question..."
              disabled={loading}
            />
            <button
              onClick={handleSendMessage}
              disabled={loading || !input.trim()}
              className="send-btn"
            >
              <SendIcon />
            </button>
          </div>
        </div>
      )}

      {/* 🟢 STYLES with MOBILE RESPONSIVENESS */}
      <style>{`
        .chat-widget-wrapper { position: fixed; bottom: 25px; right: 25px; z-index: 9999; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        
        .chat-toggle-btn { width: 65px; height: 65px; border-radius: 50%; background: linear-gradient(135deg, #2ecc71, #27ae60); border: none; cursor: pointer; box-shadow: 0 8px 25px rgba(46, 204, 113, 0.4); display: flex; align-items: center; justify-content: center; transition: transform 0.2s; }
        .chat-toggle-btn:hover { transform: scale(1.05); }
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
        .close-btn { background: rgba(255,255,255,0.2); border: none; border-radius: 50%; width: 32px; height: 32px; display: flex; align-items: center; justify-content: center; cursor: pointer; color: #1e2a38; }
        
        .chat-messages { flex: 1; padding: 20px; overflow-y: auto; display: flex; flex-direction: column; gap: 15px; background-color: #151e29; }
        .message-row { display: flex; gap: 10px; align-items: flex-end; animation: fadeIn 0.3s ease; }
        .message-row.user { justify-content: flex-end; }
        .bot-avatar { width: 30px; height: 30px; background: #2c3e50; border-radius: 50%; display: flex; align-items: center; justify-content: center; color: #2ecc71; flex-shrink: 0; }
        
        .message-bubble { padding: 12px 16px; border-radius: 18px; max-width: 80%; line-height: 1.5; font-size: 14px; position: relative; box-shadow: 0 1px 2px rgba(0,0,0,0.2); }
        .message-bubble.assistant { background-color: #2c3e50; color: #e2e8f0; border-bottom-left-radius: 4px; }
        .message-bubble.user { background-color: #2ecc71; color: #1e2a38; border-bottom-right-radius: 4px; font-weight: 500; }
        
        .message-bubble.typing { padding: 15px 20px; display: flex; align-items: center; gap: 4px; min-height: 40px; }
        .typing-dot { width: 8px; height: 8px; background-color: #94a3b8; border-radius: 50%; animation: bounce 1.4s infinite ease-in-out both; }
        .typing-dot:nth-child(1) { animation-delay: -0.32s; }
        .typing-dot:nth-child(2) { animation-delay: -0.16s; }
        
        .chat-input-area { padding: 20px; background-color: #1e2a38; border-top: 1px solid #2c3e50; display: flex; gap: 10px; align-items: center; }
        .chat-input-area input { flex: 1; padding: 12px 15px; border-radius: 25px; border: 1px solid #2c3e50; background-color: #151e29; color: white; font-size: 14px; }
        .chat-input-area input:focus { outline: none; border-color: #2ecc71; }
        .send-btn { width: 45px; height: 45px; background-color: #2ecc71; color: #1e2a38; border: none; border-radius: 50%; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: transform 0.2s; }
        .send-btn:hover:not(:disabled) { transform: scale(1.1); }
        .send-btn:disabled { background-color: #2c3e50; color: #94a3b8; }
        
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(46, 204, 113, 0.7); } 70% { box-shadow: 0 0 0 15px rgba(46, 204, 113, 0); } 100% { box-shadow: 0 0 0 0 rgba(46, 204, 113, 0); } }
        @keyframes bounce { 0%, 80%, 100% { transform: scale(0); } 40% { transform: scale(1); } }
        @keyframes slideUp { from { opacity: 0; transform: translateY(30px) scale(0.95); } to { opacity: 1; transform: translateY(0) scale(1); } }
        @keyframes fadeIn { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }
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
