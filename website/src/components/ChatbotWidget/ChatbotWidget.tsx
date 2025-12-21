/// <reference types="vite/client" />
import React, { useEffect, useRef, useState } from "react";
import "./ChatbotWidget.css"; // हम नीचे CSS भी देंगे

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<any[]>([]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string>("");
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const API_BASE_URL =
    typeof window !== "undefined" && import.meta?.env?.VITE_BACKEND_URL
      ? import.meta.env.VITE_BACKEND_URL
      : "http://localhost:8000";

  const toggleChatbot = () => setIsOpen((prev) => !prev);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Selected Text Detection
  useEffect(() => {
    const handleTextSelection = () => {
      if (typeof window === "undefined") return;

      const selection = window.getSelection();
      const text = selection?.toString().trim() || "";

      if (text.length > 15) {
        setSelectedText(text);
        if (!isOpen) setIsOpen(true);
        // Input auto suggestion
        if (!inputValue.trim()) {
          setInputValue("Explain this:");
        }
      } else if (selectedText && text.length === 0) {
        setSelectedText("");
      }
    };

    document.addEventListener("mouseup", handleTextSelection);
    document.addEventListener("touchend", handleTextSelection);

    return () => {
      document.removeEventListener("mouseup", handleTextSelection);
      document.removeEventListener("touchend", handleTextSelection);
    };
  }, [isOpen, inputValue, selectedText]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: "user",
      timestamp: new Date().toISOString(),
    };

    setMessages((prev) => [...prev, userMessage]);
    const questionToSend = inputValue;
    setInputValue("");
    setIsLoading(true);

    try {
      const requestBody = {
        question: questionToSend,
        selected_text: selectedText || null,
        mode: null,
      };

      const response = await fetch(`${API_BASE_URL}/agent/query`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) throw new Error(`Error: ${response.status}`);

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer || "No response.",
        sender: "bot",
        sources:
          data.sources?.map((s: any) => s.content || s.document_id) || [],
        timestamp: new Date().toISOString(),
      };

      setMessages((prev) => [...prev, botMessage]);
      setSelectedText(""); // Clear after use
    } catch (error) {
      console.error(error);
      setMessages((prev) => [
        ...prev,
        {
          id: Date.now() + 1,
          text: "Sorry, something went wrong. Try again.",
          sender: "bot",
          isError: true,
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Button */}
      {!isOpen && (
        <button
          onClick={toggleChatbot}
          className="chatbot-fab"
          aria-label="Open CourseBot"
        >
          <span className="fab-icon">💬</span>
          <span className="fab-pulse"></span>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-card">
            {/* Header */}
            <div className="chatbot-header">
              <div className="header-info">
                <h3>CourseBot</h3>
                <p>Ask anything about Physical AI & Humanoid Robotics</p>
              </div>
              <button
                onClick={toggleChatbot}
                className="close-btn"
                aria-label="Close"
              >
                ✕
              </button>
            </div>

            {/* Selected Text Preview */}
            {selectedText && (
              <div className="selected-text-preview">
                <strong>📝 Selected Text:</strong>
                <p>
                  {selectedText.length > 200
                    ? selectedText.substring(0, 200) + "..."
                    : selectedText}
                </p>
              </div>
            )}

            {/* Messages Area */}
            <div className="messages-area">
              {messages.length === 0 && !selectedText ? (
                <div className="welcome-box">
                  <h4>Welcome to CourseBot! 👋</h4>
                  <p>
                    Highlight any text on the page — I'll explain it instantly!
                  </p>
                  <p>Or ask anything about:</p>
                  <ul>
                    <li>ROS 2 & Navigation</li>
                    <li>Gazebo / Unity Simulation</li>
                    <li>NVIDIA Isaac Platform</li>
                    <li>VLA Models & Embodiment</li>
                  </ul>
                </div>
              ) : (
                <>
                  {messages.map((msg) => (
                    <div key={msg.id} className={`message ${msg.sender}`}>
                      <div className="bubble">
                        <p>{msg.text}</p>
                        {msg.sender === "bot" && msg.sources?.length > 0 && (
                          <details className="sources">
                            <summary>Sources ({msg.sources.length})</summary>
                            <ul>
                              {msg.sources
                                .slice(0, 4)
                                .map((src: string, i: number) => (
                                  <li key={i}>{src}</li>
                                ))}
                              {msg.sources.length > 4 && <li>... and more</li>}
                            </ul>
                          </details>
                        )}
                      </div>
                    </div>
                  ))}

                  {isLoading && (
                    <div className="message bot">
                      <div className="bubble typing">
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  )}
                  <div ref={messagesEndRef} />
                </>
              )}
            </div>

            {/* Input Area */}
            <div className="input-area">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder={
                  selectedText
                    ? "Explain this selected text..."
                    : "Ask a question about the course..."
                }
                rows={1}
                disabled={isLoading}
                className="chat-input"
              />
              <button
                onClick={sendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="send-btn"
              >
                {isLoading ? "⟳" : "➤"}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatbotWidget;
