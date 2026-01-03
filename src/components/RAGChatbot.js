import React, { useState, useRef, useEffect } from 'react';

const RAGChatbot = () => {
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the book content!", sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Send the message to the backend API
      const response = await fetch('http://localhost:8000/api/v1/rag/book-wide', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputText,
          session_id: 'chat-session-' + Date.now()
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer || "I couldn't find an answer to your question.",
        sender: 'bot'
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your question. Please try again.",
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div style={{
      padding: '20px',
      maxWidth: '800px',
      margin: '0 auto',
      fontFamily: 'system-ui, -apple-system, sans-serif'
    }}>
      <h2 style={{
        textAlign: 'center',
        color: '#2e8555',
        marginBottom: '20px'
      }}>
        Book Q&A Assistant
      </h2>

      <div style={{
        border: '1px solid #ddd',
        borderRadius: '8px',
        height: '400px',
        display: 'flex',
        flexDirection: 'column',
        marginBottom: '15px',
        boxShadow: '0 2px 10px rgba(0,0,0,0.1)'
      }}>
        <div style={{
          flex: 1,
          overflowY: 'auto',
          padding: '15px',
          backgroundColor: '#fafafa'
        }}>
          {messages.map((message) => (
            <div
              key={message.id}
              style={{
                marginBottom: '10px',
                textAlign: message.sender === 'user' ? 'right' : 'left'
              }}
            >
              <div
                style={{
                  display: 'inline-block',
                  padding: '8px 12px',
                  borderRadius: '18px',
                  maxWidth: '80%',
                  backgroundColor: message.sender === 'user' ? '#2e8555' : '#e9ecef',
                  color: message.sender === 'user' ? 'white' : '#212529'
                }}
              >
                {message.text}
              </div>
            </div>
          ))}
          {isLoading && (
            <div style={{ textAlign: 'left', marginBottom: '10px' }}>
              <div
                style={{
                  display: 'inline-block',
                  padding: '8px 12px',
                  borderRadius: '18px',
                  backgroundColor: '#e9ecef',
                  color: '#212529'
                }}
              >
                Thinking...
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>
      </div>

      <div style={{ display: 'flex', gap: '10px' }}>
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book..."
          style={{
            flex: 1,
            padding: '10px',
            border: '1px solid #ddd',
            borderRadius: '4px',
            resize: 'vertical',
            minHeight: '60px',
            maxHeight: '120px'
          }}
          disabled={isLoading}
        />
        <button
          onClick={sendMessage}
          disabled={isLoading || !inputText.trim()}
          style={{
            padding: '10px 20px',
            backgroundColor: '#2e8555',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: (isLoading || !inputText.trim()) ? 'not-allowed' : 'pointer',
            opacity: (isLoading || !inputText.trim()) ? 0.6 : 1
          }}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>

      <div style={{
        marginTop: '15px',
        fontSize: '14px',
        color: '#6c757d',
        textAlign: 'center'
      }}>
        <p>This chatbot can answer questions about Physical AI, Humanoid Robotics, ROS 2, Digital Twins, AI integration, VLA systems, and more!</p>
      </div>
    </div>
  );
};

export default RAGChatbot;