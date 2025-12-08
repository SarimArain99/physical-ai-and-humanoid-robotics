import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';
import { useHistory } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ProfileContent = () => {
  const { user, updateProfile, loading } = useAuth();
  const history = useHistory();
  
  const [isEditing, setIsEditing] = useState(false);
  const [profileData, setProfileData] = useState({
    technical_background: '',
    hardware_access: '',
    learning_goals: ''
  });
  const [message, setMessage] = useState('');

  // Redirect if not logged in
  useEffect(() => {
    if (!loading && !user) {
      history.push('/');
    }
  }, [user, loading, history]);

  // Load user data
  useEffect(() => {
    if (user) {
      setProfileData({
        technical_background: user.technical_background || '',
        hardware_access: user.hardware_access || '',
        learning_goals: user.learning_goals || ''
      });
    }
  }, [user]);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setProfileData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    const result = await updateProfile(profileData);
    if (result.success) {
      setMessage('Profile updated successfully!');
      setTimeout(() => setIsEditing(false), 1000);
    } else {
      setMessage(result.message || 'Failed to update');
    }
  };

  if (loading) return <div className="profile-loading">Loading Profile...</div>;
  if (!user) return null;

  return (
    <div className="user-profile-wrapper">
      <div className="user-profile-card">
        
        {/* Header Section */}
        <div className="profile-header">
           <div className="profile-avatar">
              {user.name ? user.name.charAt(0).toUpperCase() : 'U'}
           </div>
           <div className="profile-title-box">
              <h2>{user.name || 'User'}</h2>
              <p className="profile-email">{user.email}</p>
              <span className="profile-badge">{user.proficiency || 'Beginner'}</span>
           </div>
        </div>

        {/* Message Alert */}
        {message && <div className="profile-alert">{message}</div>}

        <hr className="profile-divider"/>

        {/* View Mode */}
        {!isEditing ? (
         <div className="profile-view-mode">
            <h3>Learning Profile</h3>
            
            <div className="profile-field">
                <label>Technical Background</label>
                <div className="field-value">{user.technical_background || 'Not set'}</div>
            </div>
            
            <div className="profile-field">
                <label>Hardware Access</label>
                <div className="field-value">{user.hardware_access || 'Not set'}</div>
            </div>
            
            <div className="profile-field">
                <label>Learning Goals</label>
                <div className="field-value">{user.learning_goals || 'Not set'}</div>
            </div>

            <button className="profile-btn-primary" onClick={() => setIsEditing(true)}>
                Edit Profile
            </button>
         </div>
        ) : (
         /* Edit Mode */
         <form onSubmit={handleSubmit} className="profile-edit-form">
            <h3>Edit Profile</h3>
            
            <div className="profile-form-group">
                <label>Technical Background</label>
                <input 
                    name="technical_background" 
                    value={profileData.technical_background} 
                    onChange={handleInputChange} 
                    placeholder="e.g. CS Student, Hobbyist..." 
                />
            </div>
            
            <div className="profile-form-group">
                <label>Hardware Access</label>
                <input 
                    name="hardware_access" 
                    value={profileData.hardware_access} 
                    onChange={handleInputChange} 
                    placeholder="e.g. Arduino, Raspberry Pi, None" 
                />
            </div>
            
            <div className="profile-form-group">
                <label>Learning Goals</label>
                <input 
                    name="learning_goals" 
                    value={profileData.learning_goals} 
                    onChange={handleInputChange} 
                    placeholder="e.g. Build a walking robot" 
                />
            </div>
            
            <div className="profile-btn-group">
                <button type="submit" className="profile-btn-primary">Save Changes</button>
                <button type="button" className="profile-btn-secondary" onClick={() => setIsEditing(false)}>Cancel</button>
            </div>
         </form>
        )}
      </div>

      {/* 🟢 FORCEFUL STYLES (No Conflicts) */}
      <style>{`
        /* Page Background */
        .user-profile-wrapper {
            display: flex;
            justify-content: center;
            padding: 4rem 1rem;
            background-color: #151e29;
            min-height: 85vh;
            font-family: sans-serif;
        }

        /* The Main Card */
        .user-profile-card {
            background-color: #1E2A38 !important; /* Force Dark Blue */
            padding: 2.5rem;
            border-radius: 16px;
            width: 100%;
            max-width: 550px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.5);
            border: 1px solid #2C3E50;
            color: #ffffff;
        }

        /* Header */
        .profile-header {
            display: flex;
            align-items: center;
            gap: 20px;
            margin-bottom: 20px;
        }
        .profile-avatar {
            width: 70px;
            height: 70px;
            background: linear-gradient(135deg, #2ECC71, #27ae60);
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 32px;
            font-weight: bold;
            color: #1E2A38;
        }
        .profile-title-box h2 {
            margin: 0;
            color: #ffffff;
            font-size: 24px;
        }
        .profile-email {
            color: #94a3b8;
            margin: 4px 0 8px 0;
            font-size: 14px;
        }
        .profile-badge {
            background-color: #8b5cf6;
            color: white;
            padding: 4px 10px;
            border-radius: 12px;
            font-size: 11px;
            text-transform: uppercase;
            font-weight: bold;
            letter-spacing: 0.5px;
        }

        .profile-divider {
            border: 0;
            height: 1px;
            background: #2C3E50;
            margin: 20px 0;
        }

        /* Fields (View Mode) */
        .profile-view-mode h3, .profile-edit-form h3 {
            color: #2ECC71;
            margin-bottom: 20px;
            border-bottom: 2px solid #2ECC71;
            display: inline-block;
            padding-bottom: 5px;
        }
        
        .profile-field { margin-bottom: 18px; }
        .profile-field label {
            display: block;
            color: #94a3b8;
            font-size: 12px;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 5px;
        }
        .field-value {
            font-size: 16px;
            color: white;
            background: rgba(255,255,255,0.05);
            padding: 10px;
            border-radius: 6px;
            border: 1px solid #2C3E50;
        }

        /* Forms (Edit Mode) */
        .profile-form-group { margin-bottom: 15px; }
        .profile-form-group label {
            display: block;
            color: #94a3b8;
            font-size: 13px;
            margin-bottom: 6px;
        }
        .profile-form-group input {
            width: 100%;
            padding: 12px;
            background-color: #0f172a;
            border: 1px solid #334155;
            border-radius: 8px;
            color: white;
            font-size: 15px;
            outline: none;
            transition: border 0.2s;
        }
        .profile-form-group input:focus {
            border-color: #2ECC71;
        }

        /* Buttons */
        .profile-btn-primary {
            width: 100%;
            background-color: #2ECC71;
            color: #1E2A38;
            border: none;
            padding: 12px;
            border-radius: 8px;
            font-weight: bold;
            cursor: pointer;
            margin-top: 10px;
            font-size: 16px;
            transition: background 0.2s;
        }
        .profile-btn-primary:hover { background-color: #22c55e; }

        .profile-btn-secondary {
            background: transparent;
            color: #94a3b8;
            border: 1px solid #475569;
            padding: 10px 20px;
            border-radius: 8px;
            cursor: pointer;
            margin-top: 10px;
        }
        .profile-btn-secondary:hover { color: white; border-color: white; }
        
        .profile-btn-group { display: flex; gap: 10px; }

        .profile-alert {
            background: rgba(46, 204, 113, 0.2);
            color: #2ECC71;
            padding: 12px;
            border-radius: 8px;
            text-align: center;
            border: 1px solid #2ECC71;
            margin-bottom: 20px;
        }
        .profile-loading {
            color: white; 
            text-align: center; 
            margin-top: 50px;
        }
      `}</style>
    </div>
  );
};

const Profile = () => {
  return (
    <Layout title="My Profile">
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => <ProfileContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default Profile;