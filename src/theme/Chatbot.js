import React, { useState, useEffect, useRef } from 'react';
import Draggable from 'react-draggable';
import { Send, MessageSquare, X } from 'lucide-react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef(null);
  const [isDarkTheme, setIsDarkTheme] = useState(false);

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      const storedTheme = localStorage.getItem('theme');
      setIsDarkTheme(storedTheme === 'dark');

      const observer = new MutationObserver(() => {
        const currentTheme = document.documentElement.getAttribute('data-theme');
        setIsDarkTheme(currentTheme === 'dark');
      });

      observer.observe(document.documentElement, {
        attributes: true,
        attributeFilter: ['data-theme'],
      });

      return () => observer.disconnect();
    }
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setMessages([{ id: Date.now(), text: 'Hello! How can I help you today?', sender: 'bot' }]);
    }
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleSendMessage = () => {
    if (inputValue.trim()) {
      const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setInputValue('');

      // Simulate a bot response
      setTimeout(() => {
        const botMessage = { id: Date.now() + 1, text: `Echo: ${inputValue}`, sender: 'bot' };
        setMessages((prevMessages) => [...prevMessages, botMessage]);
      }, 1000);
    }
  };

  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  return (
    <div className={`chatbot-container ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
      {!isOpen && (
        <button onClick={toggleChat} className="chatbot-fab">
          <MessageSquare size={24} />
        </button>
      )}

      {isOpen && (
        <Draggable handle=".chatbot-header">
          <div className="chatbot-window">
            <div className="chatbot-header">
              <span>AI Assistant</span>
              <button onClick={toggleChat} className="chatbot-close-btn">
                <X size={20} />
              </button>
            </div>
            <div className="chatbot-messages">
              {messages.map((message) => (
                <div key={message.id} className={`chatbot-message ${message.sender}`}>
                  {message.text}
                </div>
              ))}
              <div ref={messagesEndRef} />
            </div>
            <div className="chatbot-input-area">
              <input
                type="text"
                placeholder="Type your message..."
                value={inputValue}
                onChange={handleInputChange}
                onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
              />
              <button onClick={handleSendMessage}>
                <Send size={20} />
              </button>
            </div>
          </div>
        </Draggable>
      )}
    </div>
  );
};

export default Chatbot;