import React, { useState, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { Sparkles, X, Loader2, RefreshCw } from 'lucide-react';
import './AIAgentPanel.css';

const API_BASE = 'http://localhost:8000';

const AIAgentPanel = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [response, setResponse] = useState('');
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    // Automatically analyze page when panel opens
    useEffect(() => {
        if (isOpen && !response && !loading) {
            analyzeCurrentPage();
        }
    }, [isOpen]);

    const analyzeCurrentPage = async () => {
        if (!ExecutionEnvironment.canUseDOM) return;

        setLoading(true);
        setError('');
        setResponse('');

        try {
            const currentPage = window.location.pathname;

            // Construct a query that asks for a direct explanation of the page
            const query = "Explain the key concepts and summary of this page.";

            const requestBody = {
                query: query,
                current_page: currentPage,
                conversation_history: [] // No history needed for direct page analysis
            };

            console.log('Sending AI Agent request:', requestBody);

            const res = await fetch(`${API_BASE}/api/v1/chat`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(requestBody),
            });

            if (!res.ok) {
                const errorData = await res.json().catch(() => ({}));
                throw new Error(errorData.detail || 'Failed to get AI response');
            }

            const data = await res.json();
            setResponse(data.answer);
        } catch (err) {
            console.error('AI Agent error:', err);
            setError(err.message || 'Something went wrong');
        } finally {
            setLoading(false);
        }
    };

    return (
        <>
            {/* Trigger Button */}
            {!isOpen && (
                <button
                    onClick={() => setIsOpen(true)}
                    className="ai-agent-trigger-btn"
                    title="Ask AI Agent to explain this page"
                >
                    <Sparkles size={20} />
                    <span>AI Explain</span>
                </button>
            )}

            {/* Panel Overlay */}
            {isOpen && (
                <div className="ai-agent-panel-overlay" onClick={() => setIsOpen(false)}>
                    <div className="ai-agent-panel" onClick={(e) => e.stopPropagation()}>

                        <div className="ai-agent-panel-header">
                            <h3>
                                <Sparkles size={20} />
                                <span>AI Page Assistant</span>
                            </h3>
                            <button
                                onClick={() => setIsOpen(false)}
                                className="ai-agent-close-btn"
                                aria-label="Close"
                            >
                                <X size={20} />
                            </button>
                        </div>

                        <div className="ai-agent-content">
                            {loading ? (
                                <div className="ai-agent-loading">
                                    <Loader2 size={32} className="spinning" />
                                    <span style={{ marginLeft: '10px' }}>Analyzing page content...</span>
                                </div>
                            ) : error ? (
                                <div className="ai-agent-response" style={{ borderColor: '#ef4444' }}>
                                    <h4 style={{ color: '#ef4444' }}>Error</h4>
                                    <p>{error}</p>
                                    <button
                                        onClick={analyzeCurrentPage}
                                        className="ai-agent-submit-btn"
                                        style={{ marginTop: '10px' }}
                                    >
                                        <RefreshCw size={16} /> Retry
                                    </button>
                                </div>
                            ) : (
                                <div className="ai-agent-response">
                                    <h4>Page Analysis</h4>
                                    <div style={{ whiteSpace: 'pre-wrap' }}>{response}</div>
                                </div>
                            )}
                        </div>
                    </div>
                </div>
            )}
        </>
    );
};

export default AIAgentPanel;
