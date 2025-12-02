import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

const API_BASE = 'http://localhost:8000';

const Profile = () => {
  const { user, login } = useAuth();
  const history = useHistory();

  const [email, setEmail] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');

  const [personalizationInput, setPersonalizationInput] = useState('');
  const [personalizedContent, setPersonalizedContent] = useState('');
  const [personalizationLoading, setPersonalizationLoading] = useState(false);
  const [personalizationError, setPersonalizationError] = useState('');

  useEffect(() => {
    if (user) {
      // Load profile data from backend
      const loadProfile = async () => {
        try {
          const response = await fetch(`${API_BASE}/api/v1/profile/${user.user_id}`);
          if (response.ok) {
            const profileData = await response.json();
            setEmail(profileData.email || '');
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
        }
      };
      loadProfile();
    } else {
      history.push('/'); // auth optional; redirect home if not signed in
    }
  }, [user, history]);

  const handleUpdateProfile = async (e) => {
    e.preventDefault();
    if (!user) return;
    setMessage('');
    setError('');
    try {
      const response = await fetch(`${API_BASE}/api/v1/profile/${user.user_id}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          software_background: softwareBackground,
          hardware_background: hardwareBackground,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Profile update failed');
      }

      const updatedUser = await response.json();
      // Keep auth context in sync so agents immediately see new specialization on subsequent calls
      login({ ...user, ...updatedUser });
      setMessage('Profile updated successfully!');
    } catch (err) {
      setError(err.message);
    }
  };

  const handlePersonalize = async () => {
    if (!user || personalizationInput.trim() === '') return;

    setPersonalizationLoading(true);
    setPersonalizationError('');
    setPersonalizedContent('');

    try {
      const response = await fetch(`${API_BASE}/api/v1/personalize`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          user_id: user.user_id,
          chapter_content: personalizationInput,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Personalization failed');
      }

      const data = await response.json();
      setPersonalizedContent(data.personalized_content);
    } catch (err) {
      setPersonalizationError(err.message);
    } finally {
      setPersonalizationLoading(false);
    }
  };

  if (!user) {
    return null;
  }

  return (
    <div>
      <h2>User Profile</h2>
      <form onSubmit={handleUpdateProfile}>
        <div>
          <label>Email:</label>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            disabled
          />
        </div>
        <div>
          <label>Software Background:</label>
          <textarea
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
          />
        </div>
        <div>
          <label>Hardware Background:</label>
          <textarea
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
          />
        </div>
        {error && <p style={{ color: 'red' }}>{error}</p>}
        {message && <p style={{ color: 'green' }}>{message}</p>}
        <button type="submit">Update Profile</button>
      </form>

      <h2 style={{ marginTop: '40px' }}>Personalize Content</h2>
      <textarea
        value={personalizationInput}
        onChange={(e) => setPersonalizationInput(e.target.value)}
        placeholder="Paste any chapter content to personalize based on your profile..."
        rows="10"
        style={{ width: '100%', marginBottom: '10px' }}
      />
      <button onClick={handlePersonalize} disabled={personalizationLoading}>
        {personalizationLoading ? 'Personalizing...' : 'Personalize Content'}
      </button>
      {personalizationError && <p style={{ color: 'red' }}>{personalizationError}</p>}
      {personalizedContent && (
        <div style={{ marginTop: '20px', border: '1px solid #ccc', padding: '10px' }}>
          <h3>Personalized Content:</h3>
          <p>{personalizedContent}</p>
        </div>
      )}
    </div>
  );
};

export default Profile;
