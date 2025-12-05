import React, { useState, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useAuth } from '@site/src/contexts/AuthContext';
import ChatKit, { askQuestionFromSelection } from '@site/src/components/ChatKit/ChatKit';
import TextSelection from '@site/src/components/TextSelection/TextSelection';
import AIAgentPanel from '@site/src/components/AIAgentPanel/AIAgentPanel';


export default function DocItemWrapper(props) {
  const { user } = useAuth();
  const [isBrowser, setIsBrowser] = useState(false);
  const [notes, setNotes] = useState([]);
  const [newNote, setNewNote] = useState('');
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState('');

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      setIsBrowser(true);
    }
  }, []);

  const handleAddNote = async () => {
    if (!user || !ExecutionEnvironment.canUseDOM) return;
    if (!newNote.trim()) return;

    setSaving(true);
    setError('');

    try {
      const pageId = window.location.pathname;
      const response = await fetch('http://localhost:8000/api/v1/notes', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          user_id: user.user_id,
          page_id: pageId,
          content: newNote.trim(),
        }),
      });

      if (!response.ok) {
        const data = await response.json().catch(() => ({}));
        throw new Error(data.detail || 'Failed to save note');
      }

      const saved = await response.json();
      setNotes((prev) => [...prev, saved]);
      setNewNote('');
    } catch (e) {
      setError(e.message || 'Failed to save note');
    } finally {
      setSaving(false);
    }
  };

  if (!isBrowser) {
    return <DocItem {...props} />;
  }

  return (
    <>
      <DocItem {...props} />
      {/* Text selection handler */}
      <TextSelection onAskQuestion={askQuestionFromSelection} />
      {/* AI Agent Panel - Explains current page */}
      <AIAgentPanel />

      {/* ChatKit only shows on docs pages */}
      <ChatKit />
      {user && (
        <div style={{ marginTop: '2rem', padding: '1rem', borderTop: '1px solid #eee' }}>
          <h3>Your Notes</h3>
          <p style={{ fontSize: '0.9rem', opacity: 0.8 }}>
            Notes are stored in your Neon-backed profile and associated with this page.
          </p>
          <textarea
            value={newNote}
            onChange={(e) => setNewNote(e.target.value)}
            rows={3}
            style={{ width: '100%', margin: '0.5rem 0' }}
            placeholder="Write a personal note about this section..."
          />
          <button
            className="button button--primary button--sm"
            onClick={handleAddNote}
            disabled={saving}
          >
            {saving ? 'Saving…' : 'Save Note'}
          </button>
          {error && <p style={{ color: 'red', marginTop: '0.5rem' }}>{error}</p>}

          {notes.length > 0 && (
            <ul style={{ marginTop: '1rem' }}>
              {notes.map((note) => (
                <li key={note.id} style={{ marginBottom: '0.5rem' }}>
                  <div style={{ fontSize: '0.85rem' }}>{note.content}</div>
                </li>
              ))}
            </ul>
          )}
        </div>
      )}
    </>
  );
}


