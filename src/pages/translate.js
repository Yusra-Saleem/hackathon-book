import React, { useState } from 'react';
import Layout from '@theme/Layout';

const TranslationPage = () => {
  const [inputText, setInputText] = useState('');
  const [translatedText, setTranslatedText] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleTranslate = async () => {
    if (inputText.trim() === '') return;

    setLoading(true);
    setError('');
    setTranslatedText('');

    try {
      const response = await fetch('http://localhost:8000/api/v1/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ chapter_content: inputText }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedText(data.translated_content);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Translate">
      <div style={{ padding: '20px' }}>
        <h2>Translate Content to Urdu</h2>
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          placeholder="Enter English text to translate..."
          rows="10"
          style={{ width: '100%', marginBottom: '10px' }}
        />
        <button onClick={handleTranslate} disabled={loading}>
          {loading ? 'Translating...' : 'Translate to Urdu'}
        </button>
        {error && <p style={{ color: 'red' }}>{error}</p>}
        {translatedText && (
          <div style={{ marginTop: '20px', border: '1px solid #ccc', padding: '10px' }}>
            <h3>Translated Content:</h3>
            <p>{translatedText}</p>
          </div>
        )}
      </div>
    </Layout>
  );
};

export default TranslationPage;
