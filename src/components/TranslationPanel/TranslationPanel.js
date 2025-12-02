import React, { useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { Languages, X, Loader2 } from 'lucide-react';
import './TranslationPanel.css';

const API_BASE = 'http://localhost:8000';

const TranslationPanel = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const translatePageContent = async () => {
    if (!ExecutionEnvironment.canUseDOM) return;

    setLoading(true);
    setError('');
    setTranslatedContent('');

    try {
      // Get all text content from the main content area
      const mainContent = document.querySelector('.markdown');
      if (!mainContent) {
        throw new Error('Could not find page content');
      }

      // Extract text content (excluding code blocks, buttons, etc.)
      const textContent = mainContent.innerText || mainContent.textContent || '';
      
      if (!textContent.trim()) {
        throw new Error('No content found to translate');
      }

      // Limit content size (first 3000 chars to avoid API limits)
      const contentToTranslate = textContent.substring(0, 3000);
      if (textContent.length > 3000) {
        console.log('Content truncated to 3000 characters for translation');
      }

      const response = await fetch(`${API_BASE}/api/v1/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ chapter_content: contentToTranslate }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
    } catch (err) {
      setError(err.message || 'Translation failed');
      console.error('Translation error:', err);
    } finally {
      setLoading(false);
    }
  };

  const translateSelectedText = async () => {
    if (!ExecutionEnvironment.canUseDOM) return;

    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (!selectedText) {
      setError('Please select some text to translate');
      return;
    }

    setLoading(true);
    setError('');
    setTranslatedContent('');

    try {
      const response = await fetch(`${API_BASE}/api/v1/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ chapter_content: selectedText }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      // Clear selection
      selection.removeAllRanges();
    } catch (err) {
      setError(err.message || 'Translation failed');
      console.error('Translation error:', err);
    } finally {
      setLoading(false);
    }
  };

  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  return (
    <>
      {/* Floating Translate Button */}
      <button
        onClick={() => setIsOpen(true)}
        className="translation-toggle-btn"
        title="Translate to Urdu"
        aria-label="Translate to Urdu"
      >
        <Languages size={20} />
        <span>اردو</span>
      </button>

      {/* Translation Panel */}
      {isOpen && (
        <div className="translation-panel-overlay" onClick={() => setIsOpen(false)}>
          <div className="translation-panel" onClick={(e) => e.stopPropagation()}>
            <div className="translation-panel-header">
              <h3>
                <Languages size={20} />
                <span>Translate to Urdu (اردو)</span>
              </h3>
              <button
                onClick={() => {
                  setIsOpen(false);
                  setTranslatedContent('');
                  setError('');
                }}
                className="translation-close-btn"
                aria-label="Close"
              >
                <X size={20} />
              </button>
            </div>

            <div className="translation-panel-actions">
              <button
                onClick={translatePageContent}
                disabled={loading}
                className="translation-action-btn"
              >
                {loading ? (
                  <>
                    <Loader2 size={16} className="spinning" />
                    <span>Translating...</span>
                  </>
                ) : (
                  <>
                    <Languages size={16} />
                    <span>Translate Entire Page</span>
                  </>
                )}
              </button>
              <button
                onClick={translateSelectedText}
                disabled={loading}
                className="translation-action-btn"
              >
                {loading ? (
                  <>
                    <Loader2 size={16} className="spinning" />
                    <span>Translating...</span>
                  </>
                ) : (
                  <>
                    <Languages size={16} />
                    <span>Translate Selected Text</span>
                  </>
                )}
              </button>
            </div>

            {error && (
              <div className="translation-error">
                <p>{error}</p>
              </div>
            )}

            {translatedContent && (
              <div className="translation-content">
                <div className="translation-content-header">
                  <h4>Translated Content (اردو ترجمہ):</h4>
                </div>
                <div className="translation-text" dir="rtl">
                  {translatedContent}
                </div>
                <button
                  onClick={() => {
                    navigator.clipboard.writeText(translatedContent);
                    alert('Translated text copied to clipboard!');
                  }}
                  className="translation-copy-btn"
                >
                  Copy to Clipboard
                </button>
              </div>
            )}
          </div>
        </div>
      )}
    </>
  );
};

export default TranslationPanel;

