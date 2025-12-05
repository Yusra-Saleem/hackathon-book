import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import { User, Mail, Code, Cpu, Save, Loader2, AlertCircle, CheckCircle2, Sparkles, LogOut } from 'lucide-react';
import './Profile.css';

const API_BASE = 'http://localhost:8000';

const Profile = () => {
  const { user, login, logout } = useAuth();
  const history = useHistory();

  const [email, setEmail] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);

  const [personalizationInput, setPersonalizationInput] = useState('');
  const [personalizedContent, setPersonalizedContent] = useState('');
  const [personalizationLoading, setPersonalizationLoading] = useState(false);
  const [personalizationError, setPersonalizationError] = useState('');

  useEffect(() => {
    if (!user) {
      history.push('/login');
      return;
    }

    const loadProfile = async () => {
      setLoading(true);
      try {
        const userId = user.user_id || localStorage.getItem('user_id');
        if (!userId) {
          throw new Error('User ID not found');
        }

        const token = localStorage.getItem('token');
        const response = await fetch(`${API_BASE}/api/v1/profile/${userId}`, {
          headers: token ? {
            'Authorization': `Bearer ${token}`,
          } : {},
        });

        if (response.ok) {
          const profileData = await response.json();
          setEmail(profileData.email || user.email || '');
          setSoftwareBackground(profileData.software_background || '');
          setHardwareBackground(profileData.hardware_background || '');
        } else {
          // Fallback to user data from context
          setEmail(user.email || '');
          setSoftwareBackground(user.software_background || '');
          setHardwareBackground(user.hardware_background || '');
        }
      } catch (err) {
        console.error('Error loading profile:', err);
        // Fallback to user data from context
        setEmail(user.email || '');
        setSoftwareBackground(user.software_background || '');
        setHardwareBackground(user.hardware_background || '');
      } finally {
        setLoading(false);
      }
    };

    loadProfile();
  }, [user, history]);

  const handleUpdateProfile = async (e) => {
    e.preventDefault();
    if (!user) return;
    
    setMessage('');
    setError('');
    setSaving(true);

    try {
      const userId = user.user_id || localStorage.getItem('user_id');
      const token = localStorage.getItem('token');

      const response = await fetch(`${API_BASE}/api/v1/profile/${userId}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          ...(token ? { 'Authorization': `Bearer ${token}` } : {}),
        },
        body: JSON.stringify({
          software_background: softwareBackground || 'none',
          hardware_background: hardwareBackground || 'none',
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Profile update failed');
      }

      const updatedUser = await response.json();
      login({ ...user, ...updatedUser });
      setMessage('Profile updated successfully!');
      
      // Clear message after 3 seconds
      setTimeout(() => setMessage(''), 3000);
    } catch (err) {
      console.error('Update error:', err);
      if (err.message.includes('Failed to fetch')) {
        setError('Cannot connect to server. Please make sure the backend is running.');
      } else {
        setError(err.message || 'Failed to update profile');
      }
    } finally {
      setSaving(false);
    }
  };

  const handlePersonalize = async () => {
    if (!user || personalizationInput.trim() === '') return;

    setPersonalizationLoading(true);
    setPersonalizationError('');
    setPersonalizedContent('');

    try {
      const userId = user.user_id || localStorage.getItem('user_id');
      const response = await fetch(`${API_BASE}/api/v1/personalize`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          user_id: userId,
          chapter_content: personalizationInput,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Personalization failed');
      }

      const data = await response.json();
      setPersonalizedContent(data.personalized_content);
    } catch (err) {
      console.error('Personalization error:', err);
      if (err.message.includes('Failed to fetch')) {
        setPersonalizationError('Cannot connect to server. Please make sure the backend is running.');
      } else {
        setPersonalizationError(err.message || 'Personalization failed');
      }
    } finally {
      setPersonalizationLoading(false);
    }
  };

  const handleLogout = () => {
    logout();
    localStorage.removeItem('token');
    localStorage.removeItem('user_id');
    history.push('/');
  };

  if (loading) {
    return (
      <div className="profile-container">
        <div className="profile-loading">
          <Loader2 className="spinning" size={32} />
          <p>Loading profile...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="profile-container">
      <div className="profile-card">
        <div className="profile-header">
          <div className="profile-avatar-wrapper">
            <User className="profile-avatar-icon" />
          </div>
          <div className="profile-header-content">
            <h1 className="profile-title">User Profile</h1>
            <p className="profile-subtitle">Manage your account and preferences</p>
          </div>
          <button onClick={handleLogout} className="logout-button" title="Logout">
            <LogOut size={20} />
          </button>
        </div>

        <form onSubmit={handleUpdateProfile} className="profile-form">
          <div className="form-section">
            <h2 className="section-title">
              <Mail size={20} />
              Account Information
            </h2>

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
                disabled
              />
            </div>
          </div>

          <div className="form-section">
            <h2 className="section-title">
              <Code size={20} />
              Background Information
            </h2>

            <div className="form-group">
              <label className="form-label">
                <Code size={18} className="label-icon" />
                Software Background
              </label>
              <textarea
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value)}
                className="form-textarea"
                placeholder="e.g., Python, JavaScript, React, etc."
                rows={4}
                disabled={saving}
              />
            </div>

            <div className="form-group">
              <label className="form-label">
                <Cpu size={18} className="label-icon" />
                Hardware Background
              </label>
              <textarea
                value={hardwareBackground}
                onChange={(e) => setHardwareBackground(e.target.value)}
                className="form-textarea"
                placeholder="e.g., Arduino, Raspberry Pi, ROS2, etc."
                rows={4}
                disabled={saving}
              />
            </div>
          </div>

          {error && (
            <div className="alert alert-error">
              <AlertCircle size={20} />
              <span>{error}</span>
            </div>
          )}

          {message && (
            <div className="alert alert-success">
              <CheckCircle2 size={20} />
              <span>{message}</span>
            </div>
          )}

          <button 
            type="submit" 
            className="profile-button"
            disabled={saving}
          >
            {saving ? (
              <>
                <Loader2 className="spinning" size={20} />
                <span>Saving...</span>
              </>
            ) : (
              <>
                <Save size={20} />
                <span>Update Profile</span>
              </>
            )}
          </button>
        </form>

        <div className="personalization-section">
          <h2 className="section-title">
            <Sparkles size={20} />
            Personalize Content
          </h2>
          <p className="section-description">
            Paste any chapter content to personalize it based on your background
          </p>

          <textarea
            value={personalizationInput}
            onChange={(e) => setPersonalizationInput(e.target.value)}
            className="form-textarea personalization-input"
            placeholder="Paste chapter content here..."
            rows={6}
            disabled={personalizationLoading}
          />

          <button 
            onClick={handlePersonalize} 
            className="profile-button personalization-button"
            disabled={personalizationLoading || !personalizationInput.trim()}
          >
            {personalizationLoading ? (
              <>
                <Loader2 className="spinning" size={20} />
                <span>Personalizing...</span>
              </>
            ) : (
              <>
                <Sparkles size={20} />
                <span>Personalize Content</span>
              </>
            )}
          </button>

          {personalizationError && (
            <div className="alert alert-error">
              <AlertCircle size={20} />
              <span>{personalizationError}</span>
            </div>
          )}

          {personalizedContent && (
            <div className="personalized-content">
              <h3 className="personalized-title">Personalized Content:</h3>
              <div className="personalized-text">{personalizedContent}</div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default Profile;
