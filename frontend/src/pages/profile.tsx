import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';
import { useHistory } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ProfileContent = () => {
  const { user, updateProfile, loading } = useAuth();
  const history = useHistory();
  const [isEditing, setIsEditing] = useState(false);
  const [profileData, setProfileData] = useState({ technical_background: '', hardware_access: '', proficiency: 'beginner' });
  const [message, setMessage] = useState('');

  useEffect(() => { if (!loading && !user) history.push('/'); }, [user, loading, history]);
  useEffect(() => { if (user) setProfileData({ technical_background: user.technical_background || '', hardware_access: user.hardware_access || '', proficiency: user.proficiency || 'beginner' }); }, [user]);

  const handleInputChange = (e) => { const { name, value } = e.target; setProfileData(prev => ({ ...prev, [name]: value })); };
  const handleSubmit = async (e) => {
    e.preventDefault();
    const result = await updateProfile(profileData);
    if (result.success) { setMessage('Saved!'); setTimeout(() => setIsEditing(false), 1000); }
    else { setMessage('Failed'); }
  };

  if (!user) return null;

  // Styles follow the docs theme tokens (same surfaces, borders and
  // accent as the rest of the site) — the old hardcoded dark navy
  // made this page look like a different site next to the light docs.
  const containerStyle = { backgroundColor: 'var(--color-bg-primary)', minHeight: '100vh', padding: '50px 20px', display: 'flex', justifyContent: 'center' };
  const cardStyle = { backgroundColor: 'var(--color-bg-elevated)', padding: '30px', borderRadius: '15px', width: '100%', maxWidth: '500px', border: '1px solid var(--color-border)', color: 'var(--color-text-primary)' };
  const inputStyle = { width: '100%', padding: '10px', marginBottom: '15px', backgroundColor: 'var(--color-bg-surface)', border: '1px solid var(--color-border)', color: 'var(--color-text-primary)', borderRadius: '5px' };
  const btnStyle = { width: '100%', padding: '10px', backgroundColor: 'var(--color-accent)', color: 'var(--color-accent-ink)', border: 'none', fontWeight: 'bold', borderRadius: '5px', cursor: 'pointer', marginTop: '10px' };

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <h1 style={{ borderBottom: '1px solid var(--ifm-color-primary)', paddingBottom: '10px' }}>{user.name || 'Profile'}</h1>
        <p style={{ color: 'var(--color-text-tertiary)' }}>{user.email}</p>
        <span style={{ backgroundColor: 'var(--color-accent)', color: 'var(--color-accent-ink)', padding: '4px 8px', borderRadius: '4px', fontSize: '12px' }}>{user.proficiency === 'intermediate' ? 'Intermediate' : user.proficiency === 'pro' ? 'Pro' : 'Beginner'}</span>

        <hr style={{ borderColor: 'var(--color-border)', margin: '20px 0' }} />

        {message && <div style={{ color: 'var(--ifm-color-primary)', textAlign: 'center', marginBottom: '10px' }}>{message}</div>}

        {!isEditing ? (
          <div>
            <h3 style={{ color: 'var(--ifm-color-primary)' }}>My Details</h3>
            <p><strong>Background:</strong> {user.technical_background || 'Not set'}</p>
            <p><strong>Hardware:</strong> {user.hardware_access || 'Not set'}</p>
            <button style={btnStyle} onClick={() => setIsEditing(true)}>Edit Profile</button>
          </div>
        ) : (
          <form onSubmit={handleSubmit}>
            <label style={{ display: 'block', marginBottom: '5px', fontSize: '12px', color: 'var(--color-text-tertiary)' }}>Proficiency level</label>
            <select name="proficiency" value={profileData.proficiency} onChange={handleInputChange} style={{ ...inputStyle }}>
              <option value="beginner">Beginner (New to AI)</option>
              <option value="intermediate">Intermediate (Student)</option>
              <option value="pro">Pro (Researcher)</option>
            </select>

            <label style={{ display: 'block', marginBottom: '5px', fontSize: '12px', color: 'var(--color-text-tertiary)' }}>Software background</label>
            <input name="technical_background" value={profileData.technical_background} onChange={handleInputChange} style={inputStyle} />

            <label style={{ display: 'block', marginBottom: '5px', fontSize: '12px', color: 'var(--color-text-tertiary)' }}>Hardware access</label>
            <input name="hardware_access" value={profileData.hardware_access} onChange={handleInputChange} style={inputStyle} />

            <button type="submit" style={btnStyle}>Save Changes</button>
            <button type="button" onClick={() => setIsEditing(false)} style={{ ...btnStyle, backgroundColor: 'transparent', color: 'var(--color-text-tertiary)', border: '1px solid var(--color-border-light)' }}>Cancel</button>
          </form>
        )}
      </div>
    </div>
  );
};

export default function Profile() { return <Layout title="Profile"><BrowserOnly fallback={<div>Loading...</div>}>{() => <ProfileContent />}</BrowserOnly></Layout>; }