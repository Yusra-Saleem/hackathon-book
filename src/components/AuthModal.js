import React, { useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import './FloatingChatbot.css'; // reuse modal-friendly styles if available

const API_BASE = 'http://localhost:8000';

const TABS = {
  SIGN_IN: 'sign_in',
  SIGN_UP: 'sign_up',
};

export default function AuthModal({ isOpen, onClose, onAuthSuccess }) {
  const [activeTab, setActiveTab] = useState(TABS.SIGN_IN);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  if (!isOpen) {
    return null;
  }

  const resetForm = () => {
    setEmail('');
    setPassword('');
    setSoftwareBackground('');
    setHardwareBackground('');
    setError('');
  };

  const closeAndReset = () => {
    resetForm();
    onClose?.();
  };

  const persistAuth = (data) => {
    if (!ExecutionEnvironment.canUseDOM) return;
    if (data?.user_id) {
      localStorage.setItem('user_id', String(data.user_id));
    }
    if (data?.token) {
      localStorage.setItem('token', data.token);
    }
  };

  const handleSignIn = async (event) => {
    event.preventDefault();
    setLoading(true);
    setError('');

    try {
      const response = await fetch(`${API_BASE}/api/v1/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Sign-in failed');
      }

      const data = await response.json();
      persistAuth(data);
      onAuthSuccess?.(data);
      closeAndReset();
    } catch (err) {
      setError(err.message || 'Sign-in failed');
    } finally {
      setLoading(false);
    }
  };

  const handleSignUp = async (event) => {
    event.preventDefault();
    setLoading(true);
    setError('');

    try {
      const signupBody = {
        email,
        password,
        software_background: softwareBackground,
        hardware_background: hardwareBackground,
      };

      const response = await fetch(`${API_BASE}/api/v1/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(signupBody),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Sign-up failed');
      }

      // On successful signup, immediately sign the user in to obtain token/user_id
      await handleSignIn(event);
    } catch (err) {
      setLoading(false);
      setError(err.message || 'Sign-up failed');
      return;
    }
  };

  return (
    <div className="chatbot-backdrop">
      <div className="chatbot-window auth-modal">
        <div className="chatbot-header">
          <span>{activeTab === TABS.SIGN_IN ? 'Sign In' : 'Create Account'}</span>
          <button onClick={closeAndReset} className="chatbot-close-btn" aria-label="Close auth modal">
            ×
          </button>
        </div>

        <div className="auth-modal-tabs">
          <button
            type="button"
            className={`auth-tab ${activeTab === TABS.SIGN_IN ? 'active' : ''}`}
            onClick={() => setActiveTab(TABS.SIGN_IN)}
          >
            Sign In
          </button>
          <button
            type="button"
            className={`auth-tab ${activeTab === TABS.SIGN_UP ? 'active' : ''}`}
            onClick={() => setActiveTab(TABS.SIGN_UP)}
          >
            Sign Up
          </button>
        </div>

        {error && <div className="auth-error">{error}</div>}

        {activeTab === TABS.SIGN_IN ? (
          <form className="auth-form" onSubmit={handleSignIn}>
            <label className="auth-label">
              Email
              <input
                className="auth-input"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
              />
            </label>
            <label className="auth-label">
              Password
              <input
                className="auth-input"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
              />
            </label>
            <button className="auth-submit" type="submit" disabled={loading}>
              {loading ? 'Signing in…' : 'Sign In'}
            </button>
          </form>
        ) : (
          <form className="auth-form" onSubmit={handleSignUp}>
            <label className="auth-label">
              Email
              <input
                className="auth-input"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
              />
            </label>
            <label className="auth-label">
              Password
              <input
                className="auth-input"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
              />
            </label>
            <label className="auth-label">
              Software Background (required)
              <textarea
                className="auth-textarea"
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value)}
                required
              />
            </label>
            <label className="auth-label">
              Hardware Background (required)
              <textarea
                className="auth-textarea"
                value={hardwareBackground}
                onChange={(e) => setHardwareBackground(e.target.value)}
                required
              />
            </label>
            <button className="auth-submit" type="submit" disabled={loading}>
              {loading ? 'Creating account…' : 'Sign Up'}
            </button>
          </form>
        )}
      </div>
    </div>
  );
}


