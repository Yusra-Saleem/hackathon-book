import React, { useState, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { Languages, X, Loader2 } from 'lucide-react';
import './TranslationPanel.css';

const API_BASE = 'https://yusra-saleem123-hackathon-book-backend.hf.space';

const TranslationPanel = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  useEffect(() => {
    console.log('TranslationPanel mounted');
    console.log('ExecutionEnvironment.canUseDOM:', ExecutionEnvironment.canUseDOM);
  }, []);

  useEffect(() => {
    console.log('isOpen state changed to:', isOpen);
  }, [isOpen]);

  const translatePageContent = async () => {
    console.log('translatePageContent called');
    if (!ExecutionEnvironment.canUseDOM) {
      console.log('Not in browser environment');
      return;
    }

    setLoading(true);
    setError('');
    setTranslatedContent('');

    try {
      // Try multiple selectors to find the main content
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
          console.log(`Found content using selector: ${selector}`);
          mainContent = element;
          break;
        }
      }

      if (!mainContent) {
        console.error('Could not find any content element. Tried:', selectors);
        throw new Error('Could not find page content to translate. Please try selecting text manually.');
      }

      // Extract text content (excluding code blocks if possible, but for now just get text)
      // basic cleanup
      const textContent = mainContent.innerText || mainContent.textContent || '';
      console.log('Original text content length:', textContent.length);

      const cleanText = textContent.trim();

      if (!cleanText) {
        throw new Error('Page content appears to be empty');
      }

      console.log('Sending translation request for content length:', cleanText.length);

      const response = await fetch(`${API_BASE}/api/v1/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ chapter_content: cleanText }),
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error('Translation API error:', errorData);
        throw new Error(errorData.detail || `Translation failed with status ${response.status}`);
      }

      const data = await response.json();
      console.log('Translation received successfully');

      if (!data.translated_content) {
        throw new Error('Backend returned empty translation');
      }

      setTranslatedContent(data.translated_content);
    } catch (err) {
      console.error('Translation error details:', err);
      setError(err.message || 'Translation failed. Please check console for details.');
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

  return (
    <>
      {/* Translation Panel */}
      {(() => {
        console.log('Rendering panel check, isOpen:', isOpen);
        if (isOpen) {
          console.log('Panel SHOULD be visible now!');
        }
        return isOpen;
      })() && (
          <div
            className="translation-panel-overlay"
            onClick={() => setIsOpen(false)}
            style={{
              position: 'fixed',
              top: 0,
              left: 0,
              right: 0,
              bottom: 0,
              backgroundColor: 'rgba(0, 0, 0, 0.7)',
              zIndex: 99999,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center'
            }}
          >
            <div
              className="translation-panel"
              onClick={(e) => e.stopPropagation()}
              style={{
                backgroundColor: 'rgba(15, 23, 42, 0.98)',
                borderRadius: '12px',
                maxWidth: '800px',
                width: '90%',
                maxHeight: '80vh',
                display: 'flex',
                flexDirection: 'column',
                overflow: 'hidden',
                zIndex: 100000
              }}
            >
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

