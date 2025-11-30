import React, { useState, useEffect } from 'react';
import { useThemeContext } from '@docusaurus/theme-common';
import './FloatingChatbot.css';

function FloatingChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const { isDarkTheme } = useThemeContext();

  // Handle persistence if needed (e.g., local storage)
  useEffect(() => {
    // Logic to load/save chat state
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={`chatbot-container ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
      <button className="chatbot-toggle-button" onClick={toggleChat}>
        Chat
      </button>
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">Aina – AI Assistant</div>
          <div className="chatbot-messages">
            {/* Chat messages will go here */}
          </div>
          <div className="chatbot-input-area">
            <input type="text" placeholder="Type a message..." />
            <button>Send</button>
          </div>
        </div>
      )}
    </div>
  );
}

export default FloatingChatbot;