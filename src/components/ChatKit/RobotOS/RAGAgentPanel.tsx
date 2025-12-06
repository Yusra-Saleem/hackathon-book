import React, { useState } from 'react';
import styles from './RAGAgentPanel.module.css';

export default function RAGAgentPanel() {
  const [status, setStatus] = useState<'IDLE' | 'RETRIEVING' | 'PROCESSING'>('IDLE');
  const [messages, setMessages] = useState<Array<{ role: 'user' | 'agent'; text: string }>>([]);
  const [input, setInput] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim()) return;

    setMessages([...messages, { role: 'user', text: input }]);
    setStatus('RETRIEVING');
    setInput('');

    setTimeout(() => {
      setStatus('PROCESSING');
      setTimeout(() => {
        setStatus('IDLE');
        setMessages(prev => [...prev, {
          role: 'agent',
          text: 'AGENT_RAG: Context retrieved. Generating response...'
        }]);
      }, 1500);
    }, 1000);
  };

  return (
    <div className={styles.ragPanel}>
      <div className={styles.panelHeader}>
        <div className={styles.panelTitle}>
          <span className={styles.titleLabel}>AGENT_INTERFACE</span>
          <span className={styles.titleValue}>RAG_QUERY_SYSTEM</span>
        </div>
        <div className={styles.statusIndicator}>
          <span className={styles.statusDot} data-status={status}></span>
          <span className={styles.statusText}>
            {status === 'IDLE' && 'AGENT_RAG: Ready'}
            {status === 'RETRIEVING' && 'AGENT_RAG: Retrieving context...'}
            {status === 'PROCESSING' && 'AGENT_RAG: Processing...'}
          </span>
        </div>
      </div>

      <div className={styles.chatContainer}>
        {messages.length === 0 ? (
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>▸</div>
            <p>Query the textbook knowledge base</p>
          </div>
        ) : (
          <div className={styles.messages}>
            {messages.map((msg, idx) => (
              <div key={idx} className={styles.message} data-role={msg.role}>
                <span className={styles.messagePrefix}>
                  {msg.role === 'user' ? 'USER' : 'AGENT_RAG'}
                </span>
                <span className={styles.messageText}>{msg.text}</span>
              </div>
            ))}
          </div>
        )}
      </div>

      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Enter query..."
          className={styles.input}
          disabled={status !== 'IDLE'}
        />
        <button type="submit" className={styles.submitButton} disabled={status !== 'IDLE'}>
          EXECUTE
        </button>
      </form>
    </div>
  );
}

