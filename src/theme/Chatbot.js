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

  const handleSendMessage = async () => {
    if (!inputValue.trim()) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    const currentInput = inputValue;
    setInputValue('');

    try {
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: currentInput }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Network error' }));
        throw new Error(errorData.detail || 'Failed to get response');
      }

      const data = await response.json();
      const botMessage = { 
        id: Date.now() + 1, 
        text: data.answer || 'No response received', 
        sender: 'bot' 
      };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { 
        id: Date.now() + 1, 
        text: `Sorry, I couldn't connect to the AI backend. ${error.message}`, 
        sender: 'bot' 
      };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
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