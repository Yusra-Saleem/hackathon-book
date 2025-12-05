import React, { useState } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useAuth } from '../contexts/AuthContext';
import './AuthModal.css';

const TABS = {
  SIGN_IN: 'sign_in',
  SIGN_UP: 'sign_up',
};

export default function AuthModal({ isOpen, onClose, onAuthSuccess, defaultTab = 'sign_in' }) {
  const { signIn, signUp } = useAuth();
  const [activeTab, setActiveTab] = useState(defaultTab === 'sign_up' ? TABS.SIGN_UP : TABS.SIGN_IN);
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

  const persistAuth = (session) => {
    if (!ExecutionEnvironment.canUseDOM) return;
    if (session?.user?.id) {
      localStorage.setItem('user_id', String(session.user.id));
    }
    if (session?.session?.token) {
      localStorage.setItem('token', session.session.token);
    }
  };

  const handleSignIn = async (event) => {
    event.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await signIn(email, password);
      persistAuth(result);
      onAuthSuccess?.(result);
      closeAndReset();
    } catch (err) {
      setError(err.message || 'Sign-in failed');
      setLoading(false);
    }
  };

  const handleSignUp = async (event) => {
    event.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await signUp(email, password);
      persistAuth(result);
      onAuthSuccess?.(result);
      closeAndReset();
    } catch (err) {
      setError(err.message || 'Sign-up failed');
      setLoading(false);
    }
  };

  return (
    <div className="auth-modal-overlay">
      <div className="auth-modal-container">
        <div className="auth-modal-header">
          <h2>{activeTab === TABS.SIGN_IN ? 'Welcome Back' : 'Create Account'}</h2>
          <button onClick={closeAndReset} className="auth-modal-close" aria-label="Close auth modal">
            ×
          </button>
        </div>

        <div className="auth-tabs">
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

        <div className="auth-modal-body">
          {error && <div className="auth-error">{error}</div>}

          {activeTab === TABS.SIGN_IN ? (
            <form className="auth-form" onSubmit={handleSignIn}>
              <div className="auth-field">
                <label className="auth-label" htmlFor="signin-email">Email Address</label>
                <input
                  id="signin-email"
                  className="auth-input"
                  type="email"
                  placeholder="you@example.com"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>
              <div className="auth-field">
                <label className="auth-label" htmlFor="signin-password">Password</label>
                <input
                  id="signin-password"
                  className="auth-input"
                  type="password"
                  placeholder="Enter your password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>
              <button className="auth-submit" type="submit" disabled={loading}>
                {loading ? 'Signing in…' : 'Sign In'}
              </button>
            </form>
          ) : (
            <form className="auth-form" onSubmit={handleSignUp}>
              <div className="auth-field">
                <label className="auth-label" htmlFor="signup-email">Email Address</label>
                <input
                  id="signup-email"
                  className="auth-input"
                  type="email"
                  placeholder="you@example.com"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>
              <div className="auth-field">
                <label className="auth-label" htmlFor="signup-password">Password</label>
                <input
                  id="signup-password"
                  className="auth-input"
                  type="password"
                  placeholder="Create a strong password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>
              <div className="auth-field">
                <label className="auth-label" htmlFor="software-bg">Software Background</label>
                <textarea
                  id="software-bg"
                  className="auth-textarea"
                  placeholder="Tell us about your software experience..."
                  value={softwareBackground}
                  onChange={(e) => setSoftwareBackground(e.target.value)}
                  required
                />
              </div>
              <div className="auth-field">
                <label className="auth-label" htmlFor="hardware-bg">Hardware Background</label>
                <textarea
                  id="hardware-bg"
                  className="auth-textarea"
                  placeholder="Tell us about your hardware experience..."
                  value={hardwareBackground}
                  onChange={(e) => setHardwareBackground(e.target.value)}
                  required
                />
              </div>
              <button className="auth-submit" type="submit" disabled={loading}>
                {loading ? 'Creating account…' : 'Create Account'}
              </button>
            </form>
          )}
        </div>
      </div>
    </div>
  );
}


