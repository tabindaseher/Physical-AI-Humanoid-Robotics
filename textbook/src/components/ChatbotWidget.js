import React, { useState } from 'react';
import { useBaseUrl } from '@docusaurus/useBaseUrl';
import styles from './ChatbotWidget.module.css';

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you today?", sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      // In production, this will use the configured backend URL
      const BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
      const response = await fetch(`${BACKEND_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          context: 'current-page' // Would be the current page context
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          sources: data.sources
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: "Sorry, I'm having trouble connecting to the knowledge base right now. Please try again later.",
          sender: 'bot'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I'm experiencing a connection issue. Please try again later.",
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      {isOpen ? (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h4>Physical AI Assistant</h4>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.sender]}`}
              >
                <div className={styles.messageText}>{message.text}</div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <small>Sources: {message.sources.join(', ')}</small>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
          </div>
          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask about Physical AI, ROS 2, simulation, etc..."
              disabled={isLoading}
              className={styles.chatInput}
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              className={styles.sendButton}
            >
              Send
            </button>
          </form>
        </div>
      ) : (
        <button
          className={styles.chatButton}
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H16.58L15.5 19.88C15.36 20.24 15.08 20.53 14.73 20.68C14.5 20.78 14.25 20.83 14 20.83C13.75 20.83 13.5 20.78 13.27 20.68C12.92 20.53 12.64 20.24 12.5 19.88L11.23 16.5H5C4.46957 16.5 3.96086 16.2893 3.58579 15.9142C3.21071 15.5391 3 15.0304 3 14.5V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M18 9L15 12L12 9" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;