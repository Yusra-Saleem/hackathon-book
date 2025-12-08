import React, { useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { Languages, Loader2 } from 'lucide-react';
import './UrduTranslationToggle.css';

const API_BASE = 'https://yusra-saleem123-hackathon-book-backend.hf.space';

const UrduTranslationToggle = () => {
    const [isUrduMode, setIsUrduMode] = useState(false);
    const [translatedContent, setTranslatedContent] = useState('');
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const toggleUrduMode = async () => {
        if (!ExecutionEnvironment.canUseDOM) return;

        console.log('[UrduToggle] Toggle clicked, current mode:', isUrduMode);

        // If already in Urdu mode, just toggle off
        if (isUrduMode) {
            console.log('[UrduToggle] Switching back to English');
            setIsUrduMode(false);
            setTranslatedContent('');
            setError('');
            return;
        }

        // Otherwise, fetch translation
        console.log('[UrduToggle] Starting translation...');
        setLoading(true);
        setError('');

        try {
            // Find main content
            const selectors = [
                '.markdown',
                'article',
                'main',
                '.theme-doc-markdown',
                '#docusaurus_skipToContent_fallback'
            ];

            let mainContent = null;
            for (const selector of selectors) {
                const element = document.querySelector(selector);
                if (element) {
                    console.log('[UrduToggle] Found content using selector:', selector);
                    mainContent = element;
                    break;
                }
            }

            if (!mainContent) {
                throw new Error('Could not find page content to translate.');
            }

            const textContent = mainContent.innerText || mainContent.textContent || '';
            const cleanText = textContent.trim();
            console.log('[UrduToggle] Content length:', cleanText.length);

            if (!cleanText) {
                throw new Error('Page content appears to be empty');
            }

            // Call translation API
            console.log('[UrduToggle] Calling translation API...');
            const response = await fetch(`${API_BASE}/api/v1/translate`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ chapter_content: cleanText }),
            });

            console.log('[UrduToggle] API response status:', response.status);

            if (!response.ok) {
                const errorData = await response.json().catch(() => ({}));
                throw new Error(errorData.detail || `Translation failed with status ${response.status}`);
            }

            const data = await response.json();
            console.log('[UrduToggle] Translation received, length:', data.translated_content?.length);

            if (!data.translated_content) {
                throw new Error('Backend returned empty translation');
            }

            setTranslatedContent(data.translated_content);
            setIsUrduMode(true);
            console.log('[UrduToggle] State updated - isUrduMode should be true');
        } catch (err) {
            console.error('[UrduToggle] Translation error:', err);
            setError(err.message || 'Translation failed');
        } finally {
            setLoading(false);
        }
    };

    // Don't render on server-side
    if (!ExecutionEnvironment.canUseDOM) {
        return null;
    }

    return (
        <>
            {/* Toggle Button */}
            <button
                onClick={toggleUrduMode}
                disabled={loading}
                className={`urdu-toggle-btn ${isUrduMode ? 'active' : ''}`}
                title={isUrduMode ? 'Switch to English' : 'Translate to Urdu'}
            >
                {loading ? (
                    <>
                        <Loader2 size={20} className="spinning" />
                        <span>Translating...</span>
                    </>
                ) : (
                    <>
                        <Languages size={20} />
                        <span>{isUrduMode ? 'English' : 'اردو'}</span>
                    </>
                )}
            </button>

            {/* Urdu Content Overlay */}
            {isUrduMode && translatedContent && (
                <div className="urdu-content-overlay">
                    <div className="urdu-content-wrapper">
                        <div className="urdu-content" dir="rtl">
                            {translatedContent}
                        </div>
                    </div>
                </div>
            )}

            {/* Error Display */}
            {error && (
                <div className="urdu-error-toast">
                    <p>{error}</p>
                    <button onClick={() => setError('')}>×</button>
                </div>
            )}
        </>
    );
};

export default UrduTranslationToggle;
