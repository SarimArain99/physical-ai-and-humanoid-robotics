import React, { useState, useEffect, useRef } from "react";
import { useSession } from "../../auth/client";

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [showHistory, setShowHistory] = useState(false);
  const [conversations, setConversations] = useState([]);

  // ✅ Auth Session
  const { data: session } = useSession();

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const getSelectedText = () => {
    if (typeof window !== "undefined") {
      const selection = window.getSelection();
      return selection.toString().trim();
    }
    return "";
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const saveConversation = () => {
    if (messages.length > 0) {
      const newConversation = {
        id: Date.now(),
        timestamp: new Date().toLocaleString(),
        messages: [...messages],
      };
      const updatedConversations = [
        newConversation,
        ...conversations.slice(0, 9),
      ];
      setConversations(updatedConversations);
      localStorage.setItem(
        "chatbotConversations",
        JSON.stringify(updatedConversations)
      );
    }
  };

  const loadConversations = () => {
    if (typeof window !== "undefined") {
      const saved = localStorage.getItem("chatbotConversations");
      if (saved) {
        try {
          const parsed = JSON.parse(saved);
          setConversations(parsed);
        } catch (e) {
          console.error("Error loading conversations:", e);
        }
      }
    }
  };

  const loadConversation = (conversation) => {
    setMessages(conversation.messages);
    setShowHistory(false);
  };

  const clearHistory = () => {
    setConversations([]);
    localStorage.removeItem("chatbotConversations");
    setShowHistory(false);
  };

  useEffect(() => {
    loadConversations();
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]); // Scroll when messages OR loading state changes

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    if (!session) {
      setMessages((prev) => [
        ...prev,
        {
          id: Date.now(),
          text: "Please sign in to use the AI assistant.",
          sender: "bot",
        },
      ]);
      return;
    }

    const userMessage = { id: Date.now(), text: inputValue, sender: "user" };
    const selectedText = getSelectedText();

    setMessages((prev) => [...prev, userMessage]);
    setInputValue("");
    setIsLoading(true); // 🟢 Triggers the "Thinking" UI

    try {
      // ✅ Corrected Backend URL and Auth Header
      const response = await fetch(
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/chat",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${
              session?.session?.accessToken || session?.accessToken || ""
            }`,
          },
          body: JSON.stringify({
            query: inputValue,
            selected_text: selectedText,
          }),
        }
      );

      if (!response.ok) {
        if (response.status === 401)
          throw new Error("Authentication failed. Please log in.");
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: "bot",
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: error.message || "Sorry, I encountered an error.",
        sender: "bot",
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false); // 🟢 Removes the "Thinking" UI
    }
  };

  const saveAfterExchange = (newMessages) => {
    if (newMessages.length >= 2) {
      const lastMessage = newMessages[newMessages.length - 1];
      const secondLastMessage = newMessages[newMessages.length - 2];
      if (lastMessage.sender === "bot" && secondLastMessage.sender === "user") {
        saveConversation();
      }
    }
  };

  useEffect(() => {
    if (messages.length > 0) saveAfterExchange(messages);
  }, [messages]);

  const startNewConversation = () => {
    setMessages([]);
    setShowHistory(false);
  };

  const handleKeyDown = (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        showHistory ? (
          /* --- HISTORY VIEW (Dark Mode) --- */
          <div className="chat-container">
            <div className="chat-header">
              <h3>Conversation History</h3>
              <div className="chat-header-buttons">
                <button
                  className="chat-history-back"
                  onClick={() => setShowHistory(false)}
                >
                  ← Back
                </button>
                <button className="chat-clear-history" onClick={clearHistory}>
                  Clear
                </button>
                <button className="chat-close" onClick={toggleChat}>
                  ×
                </button>
              </div>
            </div>
            <div className="chat-history-list">
              {conversations.length === 0 ? (
                <div className="chat-history-empty">
                  <p>No conversation history yet.</p>
                </div>
              ) : (
                conversations.map((conversation) => (
                  <div
                    key={conversation.id}
                    className="chat-history-item"
                    onClick={() => loadConversation(conversation)}
                  >
                    <div className="chat-history-item-header">
                      <span className="chat-history-timestamp">
                        {conversation.timestamp}
                      </span>
                      <span className="chat-history-message-count">
                        {conversation.messages.length} msgs
                      </span>
                    </div>
                    <div className="chat-history-preview">
                      {conversation.messages[0]?.text.substring(0, 40)}...
                    </div>
                  </div>
                ))
              )}
            </div>
            <div className="chat-history-actions">
              <button
                className="chat-new-conversation"
                onClick={startNewConversation}
              >
                Start New Conversation
              </button>
            </div>
          </div>
        ) : (
          /* --- CHAT VIEW (Dark Mode) --- */
          <div className="chat-container">
            <div className="chat-header">
              <h3>Physical AI Assistant</h3>
              <div className="chat-header-buttons">
                <button
                  className="chat-history-btn"
                  onClick={() => setShowHistory(true)}
                >
                  📜 History
                </button>
                <button className="chat-close" onClick={toggleChat}>
                  ×
                </button>
              </div>
            </div>
            <div className="chat-messages">
              {messages.length === 0 ? (
                <div className="chat-welcome">
                  <p>👋 Hello! I'm your AI Textbook Assistant.</p>
                  <p>Ask me anything or select text to analyze.</p>
                  {!session && (
                    <p className="auth-warning">⚠️ Please sign in to chat.</p>
                  )}
                </div>
              ) : (
                messages.map((message) => (
                  <div
                    key={message.id}
                    className={`chat-message ${message.sender}`}
                  >
                    <div className="chat-message-text">{message.text}</div>
                  </div>
                ))
              )}

              {/* 🟢 THINKING ANIMATION */}
              {isLoading && (
                <div className="chat-message bot thinking">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>
            <div className="chat-input-area">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder={
                  session ? "Ask a question..." : "Please Sign In..."
                }
                disabled={isLoading || !session}
                rows="1"
              />
              <button
                onClick={handleSendMessage}
                disabled={!inputValue.trim() || isLoading || !session}
                className="chat-send-button"
              >
                {isLoading ? "..." : "➤"}
              </button>
            </div>
          </div>
        )
      ) : (
        <button className="chat-toggle-button" onClick={toggleChat}>
          <span>🤖</span>
        </button>
      )}

      {/* 🟢 DARK THEME CSS */}
      <style jsx>{`
        /* --- Colors based on your theme --- */
        /* Background: #1E2A38 (Dark Blue/Grey) */
        /* Text: White */
        /* Accent: #2AC2A1 (Green) */

        .chat-widget {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 9999;
          font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,
            sans-serif;
        }

        /* Toggle Button */
        .chat-toggle-button {
          background-color: #2ac2a1; /* Accent Color */
          color: #1e2a38;
          border: none;
          border-radius: 50%;
          width: 60px;
          height: 60px;
          display: flex;
          align-items: center;
          justify-content: center;
          cursor: pointer;
          box-shadow: 0 4px 15px rgba(0, 0, 0, 0.4);
          font-weight: bold;
          font-size: 14px;
          transition: transform 0.2s;
        }
        .chat-toggle-button:hover {
          transform: scale(1.1);
          background-color: #27ae60;
        }

        /* Main Container */
        .chat-container {
          width: 380px;
          height: 550px;
          max-width: 90vw;
          max-height: 80vh;
          display: flex;
          flex-direction: column;
          background-color: #1e2a38; /* Main Dark BG */
          color: white;
          border-radius: 12px;
          box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
          overflow: hidden;
          border: 1px solid #2c3e50;
        }

        /* Header */
        .chat-header {
          background: #151e29; /* Slightly darker than BG */
          padding: 15px;
          display: flex;
          justify-content: space-between;
          align-items: center;
          border-bottom: 1px solid #2c3e50;
        }
        .chat-header h3 {
          margin: 0;
          font-size: 16px;
          color: #2ecc71; /* Accent color for title */
        }
        .chat-header-buttons button {
          background: transparent;
          color: #bdc3c7;
          border: none;
          cursor: pointer;
          margin-left: 8px;
          font-size: 14px;
        }
        .chat-header-buttons button:hover {
          color: white;
        }

        /* Message List */
        .chat-messages {
          flex: 1;
          padding: 15px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 12px;
          background-color: #1e2a38;
        }

        /* Welcome & Warning */
        .chat-welcome {
          text-align: center;
          color: #95a5a6;
          margin-top: 40px;
          font-size: 0.95rem;
        }
        .auth-warning {
          color: #e74c3c;
          font-weight: bold;
          margin-top: 10px;
        }

        /* Message Bubbles */
        .chat-message {
          max-width: 85%;
          padding: 10px 14px;
          border-radius: 12px;
          font-size: 14px;
          line-height: 1.5;
        }

        /* User Bubble */
        .user {
          align-self: flex-end;
          background-color: #2ecc71; /* Accent */
          color: #1e2a38; /* Dark text on green for readability */
          border-bottom-right-radius: 2px;
          font-weight: 500;
        }

        /* Bot Bubble */
        .bot {
          align-self: flex-start;
          background-color: #34495e; /* Dark Grey/Blue */
          color: white;
          border-bottom-left-radius: 2px;
        }

        /* Input Area */
        .chat-input-area {
          padding: 15px;
          background-color: #151e29;
          border-top: 1px solid #2c3e50;
          display: flex;
          align-items: center;
          gap: 10px;
        }
        .chat-input-area textarea {
          flex: 1;
          background-color: #1e2a38;
          color: white;
          border: 1px solid #2c3e50;
          border-radius: 20px;
          padding: 10px 15px;
          outline: none;
          resize: none;
          font-family: inherit;
        }
        .chat-input-area textarea:focus {
          border-color: #2ecc71;
        }
        .chat-send-button {
          background-color: #2ecc71;
          color: #1e2a38;
          border: none;
          border-radius: 50%;
          width: 36px;
          height: 36px;
          cursor: pointer;
          font-weight: bold;
          display: flex;
          align-items: center;
          justify-content: center;
          transition: background 0.2s;
        }
        .chat-send-button:disabled {
          background-color: #34495e;
          color: #7f8c8d;
          cursor: not-allowed;
        }
        .chat-send-button:hover:not(:disabled) {
          background-color: #27ae60;
        }

        /* Thinking Animation */
        .thinking {
          background-color: transparent !important;
          padding: 0;
          margin-top: 5px;
        }
        .typing-indicator {
          background-color: #34495e;
          padding: 10px 15px;
          border-radius: 12px;
          border-bottom-left-radius: 2px;
          display: inline-flex;
          align-items: center;
        }
        .typing-indicator span {
          height: 8px;
          width: 8px;
          margin: 0 2px;
          background-color: #bdc3c7;
          border-radius: 50%;
          display: block;
          animation: typing 1.4s infinite ease-in-out both;
        }
        .typing-indicator span:nth-child(1) {
          animation-delay: -0.32s;
        }
        .typing-indicator span:nth-child(2) {
          animation-delay: -0.16s;
        }

        @keyframes typing {
          0%,
          80%,
          100% {
            transform: scale(0);
          }
          40% {
            transform: scale(1);
          }
        }

        /* History View Styles */
        .chat-history-list {
          background-color: #1e2a38;
        }
        .chat-history-empty {
          color: #7f8c8d;
          text-align: center;
          margin-top: 50px;
        }
        .chat-history-item {
          background-color: #2c3e50;
          margin-bottom: 8px;
          padding: 12px;
          border-radius: 8px;
          cursor: pointer;
          border: 1px solid transparent;
        }
        .chat-history-item:hover {
          border-color: #2ecc71;
        }
        .chat-history-item-header {
          display: flex;
          justify-content: space-between;
          font-size: 12px;
          color: #95a5a6;
          margin-bottom: 5px;
        }
        .chat-history-preview {
          font-size: 13px;
          color: #ecf0f1;
        }
        .chat-new-conversation {
          background-color: #2ecc71;
          color: #1e2a38;
          border: none;
          padding: 10px;
          width: 100%;
          border-radius: 6px;
          font-weight: bold;
          cursor: pointer;
          margin-top: 10px;
        }
      `}</style>
    </div>
  );
};

export default ChatWidget;
