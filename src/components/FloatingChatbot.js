import React, { useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const FloatingChatbot = () => {
  const [isVisible, setIsVisible] = useState(true);

  // Get the base URL for navigation
  const { siteConfig } = useDocusaurusContext();

  const handleChatClick = () => {
    // Navigate to the chat page
    window.location.href = '/docs/chat';
  };

  return (
    <>
      {isVisible && (
        <div
          style={{
            position: 'fixed',
            bottom: '30px',
            right: '30px',
            zIndex: '1000',
          }}
        >
          <button
            onClick={handleChatClick}
            style={{
              width: '60px',
              height: '60px',
              borderRadius: '50%',
              backgroundColor: '#2e8555',
              color: 'white',
              border: 'none',
              cursor: 'pointer',
              fontSize: '24px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
              transition: 'all 0.3s ease',
              fontWeight: 'bold',
            }}
            onMouseOver={(e) => {
              e.target.style.transform = 'scale(1.1)';
              e.target.style.boxShadow = '0 6px 16px rgba(0, 0, 0, 0.2)';
            }}
            onMouseOut={(e) => {
              e.target.style.transform = 'scale(1)';
              e.target.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
            }}
            title="Chat with Book Assistant"
            aria-label="Open chatbot"
          >
            💬
          </button>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;