import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';

export default function Profile() {
  const { user, isAuthenticated, loading, updateProfile } = useAuth();

  if (loading) {
    return (
      <Layout title="Profile" description="Your profile page">
        <div style={{ padding: '2rem', textAlign: 'center' }}>
          <h1>Loading...</h1>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    // Not authenticated, redirect to login
    if (typeof window !== 'undefined') {
      window.location.href = '/login';
    }
    return <div>Redirecting...</div>;
  }

  return (
    <Layout title="Profile" description="Your profile information">
      <div style={{ padding: '2rem', maxWidth: '600px', margin: '2rem auto' }}>
        <h1>Your Profile</h1>
        <ProfileContent user={user} updateProfile={updateProfile} />
      </div>
    </Layout>
  );
}

function ProfileContent({ user, updateProfile }: { user: any; updateProfile: any }) {
  const [name, setName] = React.useState(user?.name || '');
  const [email, setEmail] = React.useState(user?.email || '');
  const [technicalBackground, setTechnicalBackground] = React.useState('');
  const [hardwareAccess, setHardwareAccess] = React.useState('');
  const [learningGoals, setLearningGoals] = React.useState('');
  const [isEditing, setIsEditing] = React.useState(false);
  const [success, setSuccess] = React.useState(false);
  const [error, setError] = React.useState('');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess(false);

    try {
      const profileData = {
        technicalBackground,
        hardwareAccess,
        learningGoals,
      };

      const result = await updateProfile(profileData);
      if (result.success) {
        setSuccess(true);
        setIsEditing(false);
        setTimeout(() => setSuccess(false), 3000);
      } else {
        setError(result.message);
      }
    } catch (err) {
      setError('An error occurred while updating profile');
      console.error(err);
    }
  };

  return (
    <div>
      <div style={{ marginBottom: '2rem', padding: '1rem', backgroundColor: '#f8f9fa', borderRadius: '4px' }}>
        <h2>Account Information</h2>
        <p><strong>Name:</strong> {name}</p>
        <p><strong>Email:</strong> {email}</p>
      </div>

      <form onSubmit={handleSubmit}>
        <h3>Learning Profile</h3>
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="technicalBackground" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Technical Background
          </label>
          <textarea
            id="technicalBackground"
            value={technicalBackground}
            onChange={(e) => setTechnicalBackground(e.target.value)}
            disabled={!isEditing}
            rows={3}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              resize: 'vertical',
            }}
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="hardwareAccess" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Hardware Access
          </label>
          <textarea
            id="hardwareAccess"
            value={hardwareAccess}
            onChange={(e) => setHardwareAccess(e.target.value)}
            disabled={!isEditing}
            rows={3}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              resize: 'vertical',
            }}
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label htmlFor="learningGoals" style={{ display: 'block', marginBottom: '0.5rem' }}>
            Learning Goals
          </label>
          <textarea
            id="learningGoals"
            value={learningGoals}
            onChange={(e) => setLearningGoals(e.target.value)}
            disabled={!isEditing}
            rows={3}
            style={{
              width: '100%',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              resize: 'vertical',
            }}
          />
        </div>

        {success && (
          <div style={{ color: 'green', marginBottom: '1rem' }}>
            Profile updated successfully!
          </div>
        )}

        {error && (
          <div style={{ color: 'red', marginBottom: '1rem' }}>
            {error}
          </div>
        )}

        <div style={{ display: 'flex', gap: '1rem' }}>
          {!isEditing ? (
            <button
              type="button"
              onClick={() => setIsEditing(true)}
              style={{
                padding: '0.5rem 1rem',
                backgroundColor: '#007cba',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
              }}
            >
              Edit Profile
            </button>
          ) : (
            <>
              <button
                type="submit"
                style={{
                  padding: '0.5rem 1rem',
                  backgroundColor: '#28a745',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer',
                }}
              >
                Save Changes
              </button>
              <button
                type="button"
                onClick={() => {
                  setIsEditing(false);
                  setSuccess(false);
                  setError('');
                }}
                style={{
                  padding: '0.5rem 1rem',
                  backgroundColor: '#6c757d',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer',
                }}
              >
                Cancel
              </button>
            </>
          )}
        </div>
      </form>
    </div>
  );
}