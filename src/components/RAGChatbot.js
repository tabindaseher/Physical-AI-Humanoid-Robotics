import React, { useState, useEffect, useRef } from 'react';
import { useBaseUrl } from '@docusaurus/useBaseUrl';

// Chatbot component for Docusaurus
const RAGChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the content!", sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [backendStatus, setBackendStatus] = useState('checking'); // 'checking', 'available', 'unavailable'
  const messagesEndRef = useRef(null);

  // Get backend URL from environment or use default
  const getBackendUrl = () => {
    // First check for a global configuration (runtime)
    if (typeof window !== 'undefined' && window.BACKEND_URL) {
      return window.BACKEND_URL;
    }

    // Check for Docusaurus build-time environment variables
    // Use NEXT_PUBLIC prefix for Vercel/Docusaurus environment variables
    if (typeof process !== 'undefined' && process.env) {
      const envUrl = process.env.NEXT_PUBLIC_BACKEND_URL ||
                    process.env.REACT_APP_BACKEND_URL ||
                    process.env.BACKEND_URL;
      if (envUrl && envUrl !== 'http://127.0.0.1:8000' && envUrl !== 'localhost:8000') {
        return envUrl;
      }
    }

    // Default to the deployed backend URL
    return 'https://physical-ai-backend-u8wr.vercel.app';
  };

  const BACKEND_URL = getBackendUrl();

  // Check backend health on component mount
  useEffect(() => {
    const initializeHealthCheck = async () => {
      try {
        await checkBackendHealthWithRetry();
      } catch (error) {
        console.error('Error during initial health check:', error);
        setBackendStatus('unavailable');
        setMessages(prev => [
          ...prev.filter(msg => msg.id !== 0),
          {
            id: 0,
            text: `⚠️ Error connecting to backend service at ${BACKEND_URL}. Please ensure the backend is deployed and accessible.`,
            sender: 'system',
            type: 'error'
          }
        ]);
      }
    };

    initializeHealthCheck();
  }, []);

  const checkBackendHealth = async () => {
    try {
      // Set a timeout for the health check to handle mobile connectivity issues
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 10000); // 10 second timeout

      const response = await fetch(`${BACKEND_URL}/api/v1/health`, {
        signal: controller.signal,
        method: 'GET',
        headers: {
          'Accept': 'application/json',
          'Content-Type': 'application/json',
          'Cache-Control': 'no-cache',
          'X-Requested-With': 'XMLHttpRequest'  // Help identify as API request
        }
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        const healthData = await response.json();
        setBackendStatus(healthData.status === 'healthy' || healthData.status === 'degraded' ? 'available' : 'unavailable');

        // If backend is degraded, we might still be able to use it but with limitations
        if (healthData.status === 'degraded') {
          setMessages(prev => [
            ...prev.filter(msg => msg.id !== 0), // Remove any existing warning message
            {
              id: 0,
              text: "⚠️ Backend is running but in degraded mode. Vector database may not be connected. Some features might be limited.",
              sender: 'system',
              type: 'warning'
            }
          ]);
        }
      } else {
        setBackendStatus('unavailable');
        setMessages(prev => [
          ...prev.filter(msg => msg.id !== 0),
          {
            id: 0,
            text: `⚠️ Backend service is not available at ${BACKEND_URL}. Status: ${response.status} ${response.statusText}`,
            sender: 'system',
            type: 'error'
          }
        ]);
      }
    } catch (error) {
      // Handle timeout and network errors
      if (error.name === 'AbortError') {
        setBackendStatus('unavailable');
        setMessages(prev => [
          ...prev.filter(msg => msg.id !== 0),
          {
            id: 0,
            text: `⚠️ Timeout connecting to backend service at ${BACKEND_URL}. The service may be slow to respond or unavailable.`,
            sender: 'system',
            type: 'error'
          }
        ]);
      } else {
        setBackendStatus('unavailable');
        setMessages(prev => [
          ...prev.filter(msg => msg.id !== 0),
          {
            id: 0,
            text: `⚠️ Cannot connect to backend service at ${BACKEND_URL}. Network error: ${error.message}`,
            sender: 'system',
            type: 'error'
          }
        ]);
      }
    }
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Show error if backend is not available
    if (backendStatus === 'unavailable') {
      const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
      setMessages(prev => [...prev, userMessage]);
      setInputValue('');

      const errorMessage = {
        id: Date.now() + 1,
        text: `Cannot process your request because the backend service is not available at ${BACKEND_URL}.\n\nPlease ensure the backend is deployed and accessible.`,
        sender: 'bot',
        type: 'error'
      };
      setMessages(prev => [...prev, errorMessage]);
      return;
    }

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Set a timeout for the API call to handle mobile connectivity issues
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 45000); // Increased timeout for mobile networks and longer queries

      // Send query to backend
      const response = await fetch(`${BACKEND_URL}/api/v1/rag/book-wide`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
          'Cache-Control': 'no-cache',
          'X-Requested-With': 'XMLHttpRequest',  // Help identify as API request
          'X-Client-Type': 'web-app', // Additional header to identify client type
          'X-Device-Type': /Mobi|Android/i.test(navigator.userAgent) ? 'mobile' : 'desktop' // Identify device type
        },
        body: JSON.stringify({
          query: inputValue,
          session_id: 'web-session-' + Date.now()
        }),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status} - ${response.statusText}`);
      }

      const data = await response.json();

      // Check if the response indicates the RAG service is not available
      if (data && data.answer && (data.answer.includes("RAG service is not currently available") ||
          data.answer.includes("Qdrant vector database") ||
          data.answer.includes("RAG pipeline broken"))) {
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer + `\n\n💡 Tip: The backend is deployed at ${BACKEND_URL}.`,
          sender: 'bot',
          type: 'info'
        };
        setMessages(prev => [...prev, botMessage]);
      } else if (data && data.answer) {
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer,
          sender: 'bot',
          sources: data.sources || [],
          retrieved_docs_count: data.retrieved_docs_count || 0
        };

        setMessages(prev => [...prev, botMessage]);
      } else {
        // Handle unexpected response format
        const botMessage = {
          id: Date.now() + 1,
          text: "Sorry, I received an unexpected response from the backend. Please try again.",
          sender: 'bot',
          type: 'error'
        };
        setMessages(prev => [...prev, botMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      let errorMessageText = `Error connecting to the backend at ${BACKEND_URL}: `;

      if (error.name === 'AbortError') {
        errorMessageText += "Request timed out. The backend may be slow to respond or unavailable. This is common on mobile networks. Please try again.";
      } else if (error.message && (error.message.includes('NetworkError') || error.message.includes('Failed to fetch'))) {
        errorMessageText += "Network connection failed. Please check your internet connection and try again. This is common on mobile networks.";
      } else {
        errorMessageText += error.message ? `${error.message}. Please ensure the backend is deployed and accessible.` : "An unknown error occurred. Please try again.";
      }

      const errorMessage = {
        id: Date.now() + 1,
        text: errorMessageText,
        sender: 'bot',
        type: 'error'
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

  // Function to manually refresh backend health status
  const refreshBackendStatus = () => {
    setBackendStatus('checking');
    setMessages(prev => prev.filter(msg => msg.type !== 'error' && msg.type !== 'warning'));
    setTimeout(() => {
      const refreshHealth = async () => {
        try {
          await checkBackendHealthWithRetry();
        } catch (error) {
          console.error('Error during refresh health check:', error);
          setBackendStatus('unavailable');
          setMessages(prev => [
            ...prev.filter(msg => msg.id !== 0),
            {
              id: 0,
              text: `⚠️ Error connecting to backend service at ${BACKEND_URL}. Please ensure the backend is deployed and accessible.`,
              sender: 'system',
              type: 'error'
            }
          ]);
        }
      };
      refreshHealth();
    }, 1000);
  };

  // Enhanced health check with retry logic for mobile devices
  const checkBackendHealthWithRetry = async (maxRetries = 3) => {
    // Prevent multiple concurrent health checks
    if (backendStatus === 'checking') {
      return;
    }

    setBackendStatus('checking');

    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        // Set a timeout for the health check to handle mobile connectivity issues
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 15000); // Increased timeout for mobile networks

        const response = await fetch(`${BACKEND_URL}/api/v1/health`, {
          signal: controller.signal,
          method: 'GET',
          headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json',
            'Cache-Control': 'no-cache',
            'X-Requested-With': 'XMLHttpRequest', // Help identify as API request
            'X-Client-Type': 'web-app', // Additional header to identify client type
            'X-Device-Type': /Mobi|Android/i.test(navigator.userAgent) ? 'mobile' : 'desktop' // Identify device type
          }
        });

        clearTimeout(timeoutId);

        if (response.ok) {
          const healthData = await response.json();
          setBackendStatus(healthData.status === 'healthy' || healthData.status === 'degraded' ? 'available' : 'unavailable');

          // If backend is degraded, we might still be able to use it but with limitations
          if (healthData.status === 'degraded') {
            setMessages(prev => [
              ...prev.filter(msg => msg.id !== 0), // Remove any existing warning message
              {
                id: 0,
                text: "⚠️ Backend is running but in degraded mode. Vector database may not be connected. Some features might be limited.",
                sender: 'system',
                type: 'warning'
              }
            ]);
          }
          return; // Success, exit the retry loop
        } else {
          console.warn(`Health check attempt ${attempt} failed with status: ${response.status}`);
          if (attempt === maxRetries) {
            setBackendStatus('unavailable');
            setMessages(prev => [
              ...prev.filter(msg => msg.id !== 0),
              {
                id: 0,
                text: `⚠️ Backend service is not available at ${BACKEND_URL}. Status: ${response.status} ${response.statusText}`,
                sender: 'system',
                type: 'error'
              }
            ]);
          }
        }
      } catch (error) {
        console.warn(`Health check attempt ${attempt} failed:`, error);
        if (attempt === maxRetries) {
          // Handle timeout and network errors
          if (error.name === 'AbortError') {
            setBackendStatus('unavailable');
            setMessages(prev => [
              ...prev.filter(msg => msg.id !== 0),
              {
                id: 0,
                text: `⚠️ Timeout connecting to backend service at ${BACKEND_URL}. The service may be slow to respond or unavailable. This is common on mobile networks.`,
                sender: 'system',
                type: 'error'
              }
            ]);
          } else {
            setBackendStatus('unavailable');
            setMessages(prev => [
              ...prev.filter(msg => msg.id !== 0),
              {
                id: 0,
                text: `⚠️ Cannot connect to backend service at ${BACKEND_URL}. Network error: ${error.message || 'Connection failed'}. This is common on mobile networks.`,
                sender: 'system',
                type: 'error'
              }
            ]);
          }
        } else {
          // Wait before retrying (exponential backoff) - longer delays for mobile
          const delay = /Mobi|Android/i.test(navigator.userAgent) ? 2000 * attempt : 1000 * attempt; // Longer delays for mobile
          await new Promise(resolve => setTimeout(resolve, delay));
        }
      }
    }
  };

  return (
    <div className="rag-chatbot">
      {/* Chatbot toggle button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: backendStatus === 'available' ? '#4F46E5' : (backendStatus === 'checking' ? '#6B7280' : '#EF4444'),
          color: 'white',
          border: 'none',
          fontSize: '24px',
          cursor: 'pointer',
          zIndex: '1000',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
          transition: 'background-color 0.3s ease'
        }}
      >
        {backendStatus === 'available' ? '💬' : (backendStatus === 'checking' ? '🔄' : '⚠️')}
      </button>

      {/* Chatbot window */}
      {isOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 10px 25px rgba(0, 0, 0, 0.2)',
            display: 'flex',
            flexDirection: 'column',
            zIndex: '1000',
            fontFamily: 'Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen, Ubuntu, Cantarell, "Open Sans", "Helvetica Neue", sans-serif'
          }}
        >
          {/* Chat header */}
          <div
            style={{
              backgroundColor: '#4F46E5',
              color: 'white',
              padding: '16px',
              borderTopLeftRadius: '12px',
              borderTopRightRadius: '12px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center' }}>
              <h3 style={{ margin: 0, fontSize: '16px', marginRight: '8px' }}>Physical AI Assistant</h3>
              <span
                style={{
                  padding: '2px 6px',
                  borderRadius: '10px',
                  fontSize: '10px',
                  backgroundColor: backendStatus === 'available' ? '#10B981' : (backendStatus === 'checking' ? '#F59E0B' : '#EF4444'),
                  color: 'white'
                }}
              >
                {backendStatus === 'available' ? 'Online' : (backendStatus === 'checking' ? 'Checking...' : 'Offline')}
              </span>
            </div>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={refreshBackendStatus}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  borderRadius: '50%',
                  width: '24px',
                  height: '24px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center'
                }}
                title="Refresh connection status"
              >
                🔄
              </button>
              <button
                onClick={() => setIsOpen(false)}
                style={{
                  background: 'none',
                  border: 'none',
                  color: 'white',
                  fontSize: '18px',
                  cursor: 'pointer'
                }}
              >
                ×
              </button>
            </div>
          </div>

          {/* Messages container */}
          <div
            style={{
              flex: 1,
              padding: '16px',
              overflowY: 'auto',
              backgroundColor: '#f9fafb'
            }}
          >
            {messages.map((message) => {
              // Skip the initial welcome message if we have a system error/warning message
              if (message.id === 1 && messages.some(m => m.type)) return null;

              return (
                <div
                  key={message.id}
                  style={{
                    marginBottom: '12px',
                    display: 'flex',
                    justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start'
                  }}
                >
                  <div
                    style={{
                      maxWidth: '80%',
                      padding: '10px 14px',
                      borderRadius: '18px',
                      backgroundColor: message.sender === 'user' ? '#4F46E5' :
                                     message.sender === 'system' ? (message.type === 'error' ? '#FEF2F2' : '#FFFBEB') :
                                     '#e5e7eb',
                      color: message.sender === 'user' ? 'white' :
                           message.sender === 'system' ? (message.type === 'error' ? '#DC2626' : '#92400E') :
                           '#374151',
                      fontSize: '14px',
                      lineHeight: '1.4',
                      border: message.sender === 'system' ? (message.type === 'error' ? '1px solid #FECACA' : '1px solid #F59E0B') : 'none'
                    }}
                  >
                    {message.text}
                    {message.sources && message.sources.length > 0 && message.sender === 'bot' && (
                      <div style={{ marginTop: '8px', fontSize: '12px' }}>
                        <strong>Sources ({message.retrieved_docs_count || message.sources.length}):</strong>
                        <ul style={{ margin: '4px 0 0 0', padding: '0 0 0 16px' }}>
                          {message.sources.slice(0, 3).map((source, index) => (
                            <li key={index} style={{ marginBottom: '2px' }}>
                              {typeof source === 'object' && source.location?.source_file ?
                                source.location.source_file.replace('../../../docs/', '') :
                                (typeof source === 'object' && source.text_preview ?
                                  source.text_preview.substring(0, 50) + '...' :
                                  'Book content')}
                              {typeof source === 'object' && source.relevance_score && (
                                <span style={{ fontSize: '10px', opacity: 0.7, marginLeft: '4px' }}>
                                  (Score: {(source.relevance_score * 100).toFixed(1)}%)
                                </span>
                              )}
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}
                  </div>
                </div>
              );
            })}
            {isLoading && (
              <div
                style={{
                  marginBottom: '12px',
                  display: 'flex',
                  justifyContent: 'flex-start'
                }}
              >
                <div
                  style={{
                    maxWidth: '80%',
                    padding: '10px 14px',
                    borderRadius: '18px',
                    backgroundColor: '#e5e7eb',
                    color: '#374151',
                    fontSize: '14px'
                  }}
                >
                  🤔 Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <div
            style={{
              padding: '12px',
              backgroundColor: 'white',
              borderBottomLeftRadius: '12px',
              borderBottomRightRadius: '12px',
              borderTop: '1px solid #e5e7eb'
            }}
          >
            <div style={{ display: 'flex' }}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder={backendStatus === 'available' ? "Ask about the book..." : "Backend unavailable - check console for instructions"}
                rows={2}
                style={{
                  flex: 1,
                  padding: '10px',
                  border: '1px solid #d1d5db',
                  borderRadius: '8px',
                  resize: 'none',
                  fontSize: '14px',
                  backgroundColor: backendStatus === 'available' ? 'white' : '#FEE2E2',
                  color: backendStatus === 'available' ? 'inherit' : '#7F1D1D'
                }}
                disabled={isLoading || backendStatus !== 'available'}
              />
              <button
                onClick={sendMessage}
                disabled={isLoading || !inputValue.trim() || backendStatus !== 'available'}
                style={{
                  marginLeft: '8px',
                  padding: '10px 16px',
                  backgroundColor: isLoading || !inputValue.trim() || backendStatus !== 'available' ? '#d1d5db' : '#4F46E5',
                  color: 'white',
                  border: 'none',
                  borderRadius: '8px',
                  cursor: (isLoading || !inputValue.trim() || backendStatus !== 'available') ? 'not-allowed' : 'pointer',
                  fontSize: '14px'
                }}
              >
                Send
              </button>
            </div>
            {backendStatus !== 'available' && (
              <div style={{
                marginTop: '8px',
                fontSize: '12px',
                color: '#DC2626',
                textAlign: 'center'
              }}>
                Backend is at: {BACKEND_URL}
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;