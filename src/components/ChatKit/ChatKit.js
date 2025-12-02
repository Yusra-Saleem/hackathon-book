import React, { useState, useEffect, useRef } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { Send, MessageSquare, X, Loader2 } from 'lucide-react';
import './ChatKit.css';

const API_BASE = 'http://localhost:8000';

// Global function to send question from text selection
let globalAskQuestion = null;

export const askQuestionFromSelection = (selectedText) => {
  if (globalAskQuestion) {
    globalAskQuestion(selectedText);
  }
};

const ChatKit = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const [isDarkTheme, setIsDarkTheme] = useState(false);

  const sendMessageWithSelectedText = async (selectedText) => {
    if (!selectedText.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: `Explain this: "${selectedText}"`,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);

    try {
      let userId = null;
      let currentPage = null;
      
      if (ExecutionEnvironment.canUseDOM) {
        userId = localStorage.getItem('user_id');
        currentPage = window.location.pathname;
        if (!currentPage.startsWith('/docs/')) {
          currentPage = null;
        }
      }

      const requestBody = { 
        query: `Explain this text in detail with references: ${selectedText}`,
        selected_text: selectedText,
      };
      
      if (userId) {
        requestBody.user_id = userId;
      }
      if (currentPage) {
        requestBody.current_page = currentPage;
      }

      console.log('Sending request with selected text:', requestBody);

      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000);
      
      const response = await fetch(`${API_BASE}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
        signal: controller.signal,
      });
      
      clearTimeout(timeoutId);

      if (!response.ok) {
        let errorData;
        try {
          errorData = await response.json();
        } catch (e) {
          errorData = { detail: `HTTP ${response.status}: ${response.statusText}` };
        }
        throw new Error(errorData.detail || `HTTP ${response.status}: Failed to get response`);
      }

      const data = await response.json();

      if (!data || !data.answer) {
        throw new Error('Invalid response format from server');
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer || 'No response received',
        sender: 'bot',
        timestamp: new Date(),
        sources: data.sources || [],
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      
      let errorText = `Sorry, I couldn't process your request.`;
      
      if (error.message.includes('Failed to fetch') || error.message.includes('NetworkError')) {
        errorText = 'Cannot connect to backend server. Please make sure the backend is running on http://localhost:8000';
      } else if (error.message.includes('CORS')) {
        errorText = 'CORS error: Backend server is not allowing requests from this origin.';
      } else {
        errorText = `Error: ${error.message}`;
      }
      
      const errorMessage = {
        id: Date.now() + 1,
        text: errorText,
        sender: 'bot',
        timestamp: new Date(),
        isError: true,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  const handleAskQuestion = (selectedText) => {
    if (!selectedText || !selectedText.trim()) return;
    
    // Open chatbot if closed
    if (!isOpen) {
      setIsOpen(true);
    }
    
    // Set input and send message
    setInputValue(selectedText);
    
    // Wait a bit for UI to update, then send
    setTimeout(() => {
      sendMessageWithSelectedText(selectedText);
    }, 100);
  };

  // Set global function for text selection
  useEffect(() => {
    globalAskQuestion = handleAskQuestion;
    return () => {
      globalAskQuestion = null;
    };
  }, [isOpen, isLoading]);

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
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

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setMessages([
        {
          id: Date.now(),
          text: 'Hello! I\'m your AI assistant. How can I help you today?',
          sender: 'bot',
          timestamp: new Date(),
        },
      ]);
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue.trim(),
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    const currentInput = inputValue.trim();
    setInputValue('');
    setIsLoading(true);

    try {
      let userId = null;
      if (ExecutionEnvironment.canUseDOM) {
        userId = localStorage.getItem('user_id');
      }

      // Get current page path for context
      let currentPage = null;
      if (ExecutionEnvironment.canUseDOM) {
        currentPage = window.location.pathname;
        // Only include if it's a docs page
        if (!currentPage.startsWith('/docs/')) {
          currentPage = null;
        }
      }

      const requestBody = { query: currentInput };
      if (userId) {
        requestBody.user_id = userId;
      }
      if (currentPage) {
        requestBody.current_page = currentPage;
      }

      console.log('Sending request to:', `${API_BASE}/api/v1/chat`);
      console.log('Request body:', requestBody);

      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout
      
      const response = await fetch(`${API_BASE}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
        signal: controller.signal,
      });
      
      clearTimeout(timeoutId);

      console.log('Response status:', response.status);
      console.log('Response ok:', response.ok);

      if (!response.ok) {
        let errorData;
        try {
          errorData = await response.json();
        } catch (e) {
          errorData = { detail: `HTTP ${response.status}: ${response.statusText}` };
        }
        console.error('Error response:', errorData);
        throw new Error(errorData.detail || `HTTP ${response.status}: Failed to get response`);
      }

      const data = await response.json();
      console.log('Response data:', data);

      if (!data || !data.answer) {
        throw new Error('Invalid response format from server');
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer || 'No response received',
        sender: 'bot',
        timestamp: new Date(),
        sources: data.sources || [],
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      console.error('Error stack:', error.stack);
      
      let errorText = `Sorry, I couldn't process your request.`;
      
      if (error.message.includes('Failed to fetch') || error.message.includes('NetworkError')) {
        errorText = 'Cannot connect to backend server. Please make sure the backend is running on http://localhost:8000';
      } else if (error.message.includes('CORS')) {
        errorText = 'CORS error: Backend server is not allowing requests from this origin.';
      } else {
        errorText = `Error: ${error.message}`;
      }
      
      const errorMessage = {
        id: Date.now() + 1,
        text: errorText,
        sender: 'bot',
        timestamp: new Date(),
        isError: true,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  return (
    <div className={`chatkit-container ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
      {!isOpen && (
        <button onClick={toggleChat} className="chatkit-fab" aria-label="Open chat">
          <MessageSquare size={24} />
        </button>
      )}

      {isOpen && (
        <div className="chatkit-window">
          <div className="chatkit-header">
            <div className="chatkit-header-content">
              <div className="chatkit-avatar">AI</div>
              <div className="chatkit-header-text">
                <div className="chatkit-title">AI Assistant</div>
                <div className="chatkit-subtitle">Online</div>
              </div>
            </div>
            <button onClick={toggleChat} className="chatkit-close-btn" aria-label="Close chat">
              <X size={20} />
            </button>
          </div>

          <div className="chatkit-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`chatkit-message ${message.sender} ${message.isError ? 'error' : ''}`}
              >
                <div className="chatkit-message-bubble">
                  <div className="chatkit-message-text">{message.text}</div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="chatkit-sources">
                      <div className="chatkit-sources-label">Sources:</div>
                      <div className="chatkit-sources-list">
                        {message.sources.map((source, idx) => (
                          <span key={idx} className="chatkit-source-tag">
                            {source}
                          </span>
                        ))}
                      </div>
                    </div>
                  )}
                  <div className="chatkit-message-time">
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="chatkit-message bot">
                <div className="chatkit-message-bubble">
                  <div className="chatkit-typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatkit-input-container">
            <div className="chatkit-input-wrapper">
              <input
                ref={inputRef}
                type="text"
                className="chatkit-input"
                placeholder="Type your message..."
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                disabled={isLoading}
              />
              <button
                className="chatkit-send-btn"
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                aria-label="Send message"
              >
                {isLoading ? <Loader2 size={20} className="spinning" /> : <Send size={20} />}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatKit;

