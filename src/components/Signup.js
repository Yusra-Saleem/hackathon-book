import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { Mail, Lock, Code, Cpu, Loader2, AlertCircle, CheckCircle2, UserPlus, Eye, EyeOff, Sparkles } from 'lucide-react';
import './Signup.css';

const API_BASE = 'http://localhost:8000';

const Signup = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [loading, setLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);
  const [focusedField, setFocusedField] = useState(null);
  const history = useHistory();

  const getPasswordStrength = (pwd) => {
    if (!pwd) return { strength: 0, label: '', color: '' };
    let strength = 0;
    if (pwd.length >= 6) strength++;
    if (pwd.length >= 8) strength++;
    if (/[A-Z]/.test(pwd)) strength++;
    if (/[a-z]/.test(pwd)) strength++;
    if (/[0-9]/.test(pwd)) strength++;
    if (/[^A-Za-z0-9]/.test(pwd)) strength++;
    
    if (strength <= 2) return { strength, label: 'Weak', color: '#ef4444' };
    if (strength <= 4) return { strength, label: 'Medium', color: '#f59e0b' };
    return { strength, label: 'Strong', color: '#10b981' };
  };

  const passwordStrength = getPasswordStrength(password);

  const handleSignup = async (e) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setLoading(true);

    try {
      const payload = {
        username: email,
        email,
        full_name: '',
        password,
        software_background: softwareBackground || 'none',
        hardware_background: hardwareBackground || 'none',
      };

      const response = await fetch(`${API_BASE}/api/auth/users/`, {
        method: 'POST',
        headers: { 
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorMessage = errorData.detail || 
                           errorData.message || 
                           (Array.isArray(errorData) ? errorData.map(e => e.msg || e.message).join(', ') : 'Signup failed');
        throw new Error(errorMessage);
      }

      const data = await response.json();
      setSuccess('Account created successfully! Redirecting to login...');
      
      // Redirect to login after 1.5 seconds
      setTimeout(() => {
        history.push('/login');
      }, 1500);
    } catch (err) {
      console.error('Signup error:', err);
      
      // Better error handling
      if (err.message.includes('Failed to fetch') || err.message.includes('NetworkError')) {
        setError('Cannot connect to server. Please make sure the backend is running on http://localhost:8000');
      } else if (err.message.includes('CORS')) {
        setError('CORS error: Backend server is not allowing requests from this origin.');
      } else {
        setError(err.message || 'Signup failed. Please try again.');
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
            <UserPlus className="auth-icon" />
          </div>
          <h1 className="auth-title">Create Your Account</h1>
          <p className="auth-subtitle">Start your personalized learning journey today</p>
        </div>

        <form onSubmit={handleSignup} className="auth-form">
          <div className="form-group">
            <label className="form-label">
              <Mail size={18} className="label-icon" />
              Email Address
            </label>
            <div className="input-wrapper">
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                onFocus={() => setFocusedField('email')}
                onBlur={() => setFocusedField(null)}
                className={`form-input ${focusedField === 'email' ? 'focused' : ''} ${email ? 'has-value' : ''}`}
                placeholder="name@example.com"
                required
                disabled={loading}
              />
              {email && (
                <div className="input-indicator">
                  <CheckCircle2 size={16} />
                </div>
              )}
            </div>
          </div>

          <div className="form-group">
            <label className="form-label">
              <Lock size={18} className="label-icon" />
              Password
            </label>
            <div className="input-wrapper password-wrapper">
              <input
                type={showPassword ? 'text' : 'password'}
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                onFocus={() => setFocusedField('password')}
                onBlur={() => setFocusedField(null)}
                className={`form-input ${focusedField === 'password' ? 'focused' : ''} ${password ? 'has-value' : ''}`}
                placeholder="Create a strong password"
                required
                minLength={6}
                disabled={loading}
              />
              <button
                type="button"
                className="password-toggle"
                onClick={() => setShowPassword(!showPassword)}
                disabled={loading}
                tabIndex={-1}
              >
                {showPassword ? <EyeOff size={18} /> : <Eye size={18} />}
              </button>
            </div>
            {password && (
              <div className="password-strength">
                <div className="password-strength-bars">
                  {[1, 2, 3, 4, 5].map((i) => (
                    <div
                      key={i}
                      className={`strength-bar ${i <= passwordStrength.strength ? 'active' : ''}`}
                      style={{ backgroundColor: i <= passwordStrength.strength ? passwordStrength.color : '' }}
                    />
                  ))}
                </div>
                <span className="password-strength-label" style={{ color: passwordStrength.color }}>
                  {passwordStrength.label || 'Password strength'}
                </span>
              </div>
            )}
          </div>

          <div className="form-section-divider">
            <span className="divider-text">Optional Information</span>
          </div>

          <div className="form-row">
            <div className="form-group form-group-half">
              <label className="form-label">
                <Code size={18} className="label-icon" />
                Software Background
                <span className="optional-badge">Optional</span>
              </label>
              <div className="textarea-wrapper">
                <textarea
                  value={softwareBackground}
                  onChange={(e) => setSoftwareBackground(e.target.value)}
                  onFocus={() => setFocusedField('software')}
                  onBlur={() => setFocusedField(null)}
                  className={`form-textarea ${focusedField === 'software' ? 'focused' : ''}`}
                  placeholder="Python, JavaScript, React..."
                  rows={4}
                  disabled={loading}
                />
              </div>
              <p className="field-hint">Your software skills</p>
            </div>

            <div className="form-group form-group-half">
              <label className="form-label">
                <Cpu size={18} className="label-icon" />
                Hardware Background
                <span className="optional-badge">Optional</span>
              </label>
              <div className="textarea-wrapper">
                <textarea
                  value={hardwareBackground}
                  onChange={(e) => setHardwareBackground(e.target.value)}
                  onFocus={() => setFocusedField('hardware')}
                  onBlur={() => setFocusedField(null)}
                  className={`form-textarea ${focusedField === 'hardware' ? 'focused' : ''}`}
                  placeholder="Arduino, Raspberry Pi, ROS2..."
                  rows={4}
                  disabled={loading}
                />
              </div>
              <p className="field-hint">Your hardware skills</p>
            </div>
          </div>

          {error && (
            <div className="alert alert-error">
              <AlertCircle size={20} />
              <span>{error}</span>
            </div>
          )}

          {success && (
            <div className="alert alert-success">
              <CheckCircle2 size={20} />
              <span>{success}</span>
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
                <span>Creating Your Account...</span>
              </>
            ) : (
              <>
                <Sparkles size={20} />
                <span>Create Account</span>
              </>
            )}
          </button>

          <div className="auth-footer">
            <p>
              Already have an account?{' '}
              <a href="/login" className="auth-link">
                Sign In
              </a>
            </p>
          </div>
        </form>
      </div>
    </div>
  );
};

export default Signup;
