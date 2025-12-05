import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../contexts/AuthContext';
import { Mail, Lock, Loader2, AlertCircle, LogIn } from 'lucide-react';
import './Login.css';

const API_BASE = 'http://localhost:8000';

const Login = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const history = useHistory();
  const { login } = useAuth();

  const handleLogin = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const formBody = new URLSearchParams();
      formBody.append('username', email);
      formBody.append('password', password);

      const response = await fetch(`${API_BASE}/api/auth/token`, {
        method: 'POST',
        headers: { 
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: formBody.toString(),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorMessage = errorData.detail || 
                           errorData.message || 
                           'Invalid email or password';
        throw new Error(errorMessage);
      }

      const data = await response.json();
      
      // Store token and user info
      if (data.access_token) {
        const userId = data.user_id || email;
        
        localStorage.setItem('token', data.access_token);
        localStorage.setItem('user_id', userId);
        localStorage.setItem('user_email', email);
        
        // Update auth context
        login({ 
          token: data.access_token,
          user_id: userId,
          email: email
        });
        
        // Redirect to home
        history.push('/');
      } else {
        throw new Error('No access token received');
      }
    } catch (err) {
      console.error('Login error:', err);
      
      // Better error handling
      if (err.message.includes('Failed to fetch') || err.message.includes('NetworkError')) {
        setError('Cannot connect to server. Please make sure the backend is running on http://localhost:8000');
      } else if (err.message.includes('CORS')) {
        setError('CORS error: Backend server is not allowing requests from this origin.');
      } else {
        setError(err.message || 'Login failed. Please check your credentials.');
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-card">
        <div className="auth-header">
          <div className="auth-icon-wrapper">
            <LogIn className="auth-icon" />
          </div>
          <h1 className="auth-title">Welcome Back</h1>
          <p className="auth-subtitle">Sign in to continue your learning journey</p>
        </div>

        <form onSubmit={handleLogin} className="auth-form">
          <div className="form-group">
            <label className="form-label">
              <Mail size={18} className="label-icon" />
              Email
            </label>
            <input
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className="form-input"
              placeholder="Enter your email"
              required
              disabled={loading}
            />
          </div>

          <div className="form-group">
            <label className="form-label">
              <Lock size={18} className="label-icon" />
              Password
            </label>
            <input
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className="form-input"
              placeholder="Enter your password"
              required
              disabled={loading}
            />
          </div>

          {error && (
            <div className="alert alert-error">
              <AlertCircle size={20} />
              <span>{error}</span>
            </div>
          )}

          <button 
            type="submit" 
            className="auth-button"
            disabled={loading}
          >
            {loading ? (
              <>
                <Loader2 className="spinning" size={20} />
                <span>Signing In...</span>
              </>
            ) : (
              <>
                <LogIn size={20} />
                <span>Sign In</span>
              </>
            )}
          </button>

          <div className="auth-footer">
            <p>
              Don't have an account?{' '}
              <a href="/signup" className="auth-link">
                Sign Up
              </a>
            </p>
          </div>
        </form>
      </div>
    </div>
  );
};

export default Login;
