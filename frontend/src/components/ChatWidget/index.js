import React, { useState, useEffect, useRef } from "react";
import { useAuth } from "../Auth/AuthProvider";
import BrowserOnly from "@docusaurus/BrowserOnly";

// 1. The Logic Component
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
  }, [messages, isOpen]);

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
      // ✅ Use the Railway URL
      const response = await fetch(
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/chat",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${localStorage.getItem("auth_token") || ""}`,
          },
          // 🟢 FIX: Renamed 'message' to 'query' to match Backend
          body: JSON.stringify({
            query: input, // <--- CHANGED THIS
            selected_text: "", // <--- Added empty string for safety
          }),
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
      console.error(error);
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
        <button className="chat-toggle-btn" onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}

      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <span>🤖 AI Assistant</span>
            <button className="close-btn" onClick={() => setIsOpen(false)}>
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.map((msg, idx) => (
              <div key={idx} className={`message ${msg.role}`}>
                {msg.content}
              </div>
            ))}
            {loading && <div className="message assistant">Thinking...</div>}
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
            <button onClick={handleSendMessage} disabled={loading}>
              Send
            </button>
          </div>
        </div>
      )}

      {/* Basic CSS */}
      <style jsx>{`
        .chat-widget-wrapper {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 9999;
        }
        .chat-toggle-btn {
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background-color: #2ecc71;
          border: none;
          font-size: 30px;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
          transition: transform 0.2s;
        }
        .chat-toggle-btn:hover {
          transform: scale(1.1);
        }

        .chat-window {
          width: 350px;
          height: 500px;
          background-color: #1e2a38;
          border-radius: 12px;
          box-shadow: 0 8px 30px rgba(0, 0, 0, 0.5);
          display: flex;
          flex-direction: column;
          border: 1px solid #2c3e50;
          overflow: hidden;
        }
        .chat-header {
          padding: 15px;
          background-color: #2ecc71;
          color: #1e2a38;
          font-weight: bold;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }
        .close-btn {
          background: none;
          border: none;
          font-size: 24px;
          cursor: pointer;
          color: #1e2a38;
        }

        .chat-messages {
          flex: 1;
          padding: 15px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 10px;
        }
        .message {
          padding: 10px 14px;
          border-radius: 10px;
          max-width: 80%;
          line-height: 1.4;
          font-size: 14px;
        }
        .message.assistant {
          background-color: #2c3e50;
          color: white;
          align-self: flex-start;
          border-bottom-left-radius: 2px;
        }
        .message.user {
          background-color: #2ecc71;
          color: #1e2a38;
          align-self: flex-end;
          border-bottom-right-radius: 2px;
        }

        .chat-input-area {
          padding: 15px;
          border-top: 1px solid #2c3e50;
          display: flex;
          gap: 10px;
          background-color: #151e29;
        }
        .chat-input-area input {
          flex: 1;
          padding: 10px;
          border-radius: 6px;
          border: 1px solid #2c3e50;
          background-color: #1e2a38;
          color: white;
        }
        .chat-input-area button {
          padding: 8px 16px;
          background-color: #2ecc71;
          color: #1e2a38;
          border: none;
          border-radius: 6px;
          cursor: pointer;
          font-weight: bold;
        }
      `}</style>
    </div>
  );
};

// Safe Wrapper
const ChatWidget = () => {
  return (
    <BrowserOnly fallback={null}>{() => <ChatWidgetContent />}</BrowserOnly>
  );
};

export default ChatWidget;
