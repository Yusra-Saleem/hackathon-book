import React, { useState } from 'react';
import styles from './styles.module.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  sources?: string[];
}

export default function ChatInterface(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => setIsOpen(!isOpen);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMsg: Message = {
      id: Date.now().toString(),
      text: input,
      sender: 'user',
    };

    setMessages((prev) => [...prev, userMsg]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMsg.text }),
      });

      if (!response.ok) {
        throw new Error('Network response was not ok');
      }

      const data = await response.json();
      
      const botMsg: Message = {
        id: (Date.now() + 1).toString(),
        text: data.answer,
        sender: 'bot',
        sources: data.sources,
      };
      
      setMessages((prev) => [...prev, botMsg]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMsg: Message = {
        id: (Date.now() + 1).toString(),
        text: "Sorry, I couldn't connect to the AI backend.",
        sender: 'bot',
      };
      setMessages((prev) => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      sendMessage();
    }
  };

  return (
    <div className={`${styles.chatContainer} ${!isOpen ? styles.minimized : ''}`}>
      <div className={styles.chatHeader} onClick={toggleChat}>
        <span>AI Assistant</span>
        <span>{isOpen ? '▼' : '▲'}</span>
      </div>
      <div className={styles.chatMessages}>
        {messages.map((msg) => (
          <div
            key={msg.id}
            className={`${styles.message} ${
              msg.sender === 'user' ? styles.userMessage : styles.botMessage
            }`}
          >
            <div>{msg.text}</div>
            {msg.sources && msg.sources.length > 0 && (
              <div style={{fontSize: '0.8em', marginTop: '5px', opacity: 0.8}}>
                Sources: {msg.sources.join(', ')}
              </div>
            )}
          </div>
        ))}
        {isLoading && <div className={styles.message + ' ' + styles.botMessage}>Typing...</div>}
      </div>
      <div className={styles.inputArea}>
        <input
          className={styles.input}
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask about the book..."
        />
        <button className={styles.sendButton} onClick={sendMessage} disabled={isLoading}>
          Send
        </button>
      </div>
    </div>
  );
}
