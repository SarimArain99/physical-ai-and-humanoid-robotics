import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';
import { useHistory } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ProfileContent = () => {
  const { user, updateProfile, loading } = useAuth();
  const history = useHistory();
  const [isEditing, setIsEditing] = useState(false);
  const [profileData, setProfileData] = useState({ technical_background: '', hardware_access: '', learning_goals: '' });
  const [message, setMessage] = useState('');

  useEffect(() => { if (!loading && !user) history.push('/'); }, [user, loading, history]);
  useEffect(() => { if (user) setProfileData({ technical_background: user.technical_background || '', hardware_access: user.hardware_access || '', learning_goals: user.learning_goals || '' }); }, [user]);

  const handleInputChange = (e) => { const { name, value } = e.target; setProfileData(prev => ({ ...prev, [name]: value })); };
  const handleSubmit = async (e) => {
    e.preventDefault();
    const result = await updateProfile(profileData);
    if (result.success) { setMessage('Saved!'); setTimeout(() => setIsEditing(false), 1000); }
    else { setMessage('Failed'); }
  };

  if (!user) return null;

  // --- 🔒 LOCKED STYLES (Cannot be broken) ---
  const containerStyle = { backgroundColor: '#1E2A38', minHeight: '100vh', padding: '50px 20px', display: 'flex', justifyContent: 'center' };
  const cardStyle = { backgroundColor: '#151e29', padding: '30px', borderRadius: '15px', width: '100%', maxWidth: '500px', border: '1px solid #2ECC71', color: 'white' };
  const inputStyle = { width: '100%', padding: '10px', marginBottom: '15px', backgroundColor: '#0f172a', border: '1px solid #334155', color: 'white', borderRadius: '5px' };
  const btnStyle = { width: '100%', padding: '10px', backgroundColor: '#2ECC71', color: '#1E2A38', border: 'none', fontWeight: 'bold', borderRadius: '5px', cursor: 'pointer', marginTop: '10px' };

  return (
    <div style={containerStyle}>
      <div style={cardStyle}>
        <h1 style={{ color: 'white', borderBottom: '1px solid #2ECC71', paddingBottom: '10px' }}>{user.name || 'Profile'}</h1>
        <p style={{ color: '#94a3b8' }}>{user.email}</p>
        <span style={{ backgroundColor: '#8b5cf6', padding: '4px 8px', borderRadius: '4px', fontSize: '12px' }}>{user.proficiency || 'Beginner'}</span>

        <hr style={{ borderColor: '#334155', margin: '20px 0' }} />

        {message && <div style={{ color: '#2ECC71', textAlign: 'center', marginBottom: '10px' }}>{message}</div>}

        {!isEditing ? (
          <div>
            <h3 style={{ color: '#2ECC71' }}>My Goals</h3>
            <p><strong>Background:</strong> {user.technical_background || 'Not set'}</p>
            <p><strong>Hardware:</strong> {user.hardware_access || 'Not set'}</p>
            <p><strong>Goals:</strong> {user.learning_goals || 'Not set'}</p>
            <button style={btnStyle} onClick={() => setIsEditing(true)}>Edit Profile</button>
          </div>
        ) : (
          <form onSubmit={handleSubmit}>
            <label style={{ display: 'block', marginBottom: '5px', fontSize: '12px', color: '#94a3b8' }}>BACKGROUND</label>
            <input name="technical_background" value={profileData.technical_background} onChange={handleInputChange} style={inputStyle} />

            <label style={{ display: 'block', marginBottom: '5px', fontSize: '12px', color: '#94a3b8' }}>HARDWARE</label>
            <input name="hardware_access" value={profileData.hardware_access} onChange={handleInputChange} style={inputStyle} />

            <label style={{ display: 'block', marginBottom: '5px', fontSize: '12px', color: '#94a3b8' }}>GOALS</label>
            <input name="learning_goals" value={profileData.learning_goals} onChange={handleInputChange} style={inputStyle} />

            <button type="submit" style={btnStyle}>Save Changes</button>
            <button type="button" onClick={() => setIsEditing(false)} style={{ ...btnStyle, backgroundColor: 'transparent', color: '#94a3b8', border: '1px solid #475569' }}>Cancel</button>
          </form>
        )}
      </div>
    </div>
  );
};

export default function Profile() { return <Layout title="Profile"><BrowserOnly fallback={<div>Loading...</div>}>{() => <ProfileContent />}</BrowserOnly></Layout>; }