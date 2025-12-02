import React, { useState, useEffect } from 'react';
import { useThemeContext } from '@docusaurus/theme-common';
import './FloatingChatbot.css';
import { useAuth } from '../contexts/AuthContext'; // Import useAuth

function FloatingChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputText, setInputText] = useState('');
  const { isDarkTheme } = useThemeContext();
  const { user } = useAuth(); // Get user from AuthContext

  // Handle persistence if needed (e.g., local storage)
  useEffect(() => {
    // Logic to load/save chat state
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async () => {
    if (inputText.trim() === '') return;

    const userMessage = { text: inputText, sender: 'user' };
    setMessages(prevMessages => [...prevMessages, userMessage]);
    setInputText('');

    try {
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // You might need to add an Authorization header here if your backend requires it
          // 'Authorization': `Bearer ${user.token}`, 
        },
        body: JSON.stringify({ query: inputText }),
      });

      if (!response.ok) {
        throw new Error('Network response was not ok');
      }

      const data = await response.json();
      const aiMessage = { text: data.answer, sender: 'ai' };
      setMessages(prevMessages => [...prevMessages, aiMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { text: 'Sorry, something went wrong.', sender: 'ai' };
      setMessages(prevMessages => [...prevMessages, errorMessage]);
    }
  };

  // Only render the chatbot if the user is logged in
  if (!user) {
    return null;
  }

  return (
    <div className={`chatbot-container ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
      <button className="chatbot-toggle-button" onClick={toggleChat}>
        Chat
      </button>
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">Aina – AI Assistant</div>
          <div className="chatbot-messages">
            {messages.map((message, index) => (
              <div key={index} className={`message ${message.sender}`}>
                {message.text}
              </div>
            ))}
          </div>
          <div className="chatbot-input-area">
            <input
              type="text"
              placeholder="Type a message..."
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
            />
            <button onClick={handleSendMessage}>Send</button>
          </div>
        </div>
      )}
    </div>
  );
}

export default FloatingChatbot;