import React, { useState, useEffect, useRef } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { MessageSquare, Languages } from 'lucide-react';
import './TextSelection.css';

const API_BASE = 'http://localhost:8000';

const TextSelection = ({ onAskQuestion }) => {
  const [selectedText, setSelectedText] = useState('');
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });
  const [showButton, setShowButton] = useState(false);
  const [showTranslateButton, setShowTranslateButton] = useState(false);
  const [translating, setTranslating] = useState(false);
  const [translatedText, setTranslatedText] = useState('');
  const buttonRef = useRef(null);
  const translateButtonRef = useRef(null);

  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 0) {
        setSelectedText(text);
        
        // Get selection position
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        // Position buttons near selection
        setButtonPosition({
          top: rect.bottom + window.scrollY + 10,
          left: rect.left + window.scrollX + (rect.width / 2) - 30,
        });
        
        setShowButton(true);
        setShowTranslateButton(true);
      } else {
        setShowButton(false);
        setShowTranslateButton(false);
        setSelectedText('');
        setTranslatedText('');
      }
    };

    const handleClickOutside = (e) => {
      // Don't hide if clicking buttons or popup
      if (
        e.target.closest('.text-selection-button') ||
        e.target.closest('.translation-popup')
      ) {
        return;
      }
      
      // Hide buttons when clicking elsewhere
      setTimeout(() => {
        const selection = window.getSelection();
        if (selection.toString().trim().length === 0) {
          setShowButton(false);
          setShowTranslateButton(false);
          setSelectedText('');
          setTranslatedText('');
        }
      }, 100);
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  const handleAskClick = () => {
    if (selectedText.trim()) {
      onAskQuestion(selectedText);
      // Clear selection
      window.getSelection().removeAllRanges();
      setShowButton(false);
      setShowTranslateButton(false);
      setSelectedText('');
      setTranslatedText('');
    }
  };

  const handleTranslateClick = async () => {
    if (!selectedText.trim() || translating) return;

    setTranslating(true);
    setTranslatedText('');

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
      setTranslatedText(data.translated_content);
    } catch (err) {
      alert(`Translation failed: ${err.message}`);
      console.error('Translation error:', err);
    } finally {
      setTranslating(false);
    }
  };

  if (!ExecutionEnvironment.canUseDOM || (!showButton && !showTranslateButton)) {
    return null;
  }

  return (
    <>
      {showButton && (
        <button
          ref={buttonRef}
          className="text-selection-button"
          onClick={handleAskClick}
          style={{
            top: `${buttonPosition.top}px`,
            left: `${buttonPosition.left - 60}px`,
          }}
          title="Ask about selected text"
        >
          <MessageSquare size={16} />
          <span>Ask</span>
        </button>
      )}
      {showTranslateButton && (
        <button
          ref={translateButtonRef}
          className="text-selection-button text-selection-button-translate"
          onClick={handleTranslateClick}
          disabled={translating}
          style={{
            top: `${buttonPosition.top}px`,
            left: `${buttonPosition.left + 10}px`,
          }}
          title="Translate to Urdu"
        >
          {translating ? (
            <>
              <div className="spinner" />
              <span>...</span>
            </>
          ) : (
            <>
              <Languages size={16} />
              <span>اردو</span>
            </>
          )}
        </button>
      )}
      {translatedText && (
        <div
          className="translation-popup"
          style={{
            top: `${buttonPosition.top + 50}px`,
            left: `${buttonPosition.left - 100}px`,
          }}
        >
          <div className="translation-popup-header">
            <span>اردو ترجمہ</span>
            <button
              onClick={() => {
                setTranslatedText('');
                setShowTranslateButton(false);
                window.getSelection().removeAllRanges();
              }}
              className="translation-popup-close"
            >
              ×
            </button>
          </div>
          <div className="translation-popup-content" dir="rtl">
            {translatedText}
          </div>
          <button
            onClick={() => {
              navigator.clipboard.writeText(translatedText);
              alert('Copied to clipboard!');
            }}
            className="translation-popup-copy"
          >
            Copy
          </button>
        </div>
      )}
    </>
  );
};

export default TextSelection;

