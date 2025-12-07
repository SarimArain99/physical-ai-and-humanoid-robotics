import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';

export default function Login() {
  const { login, isAuthenticated } = useAuth();

  // If already authenticated, redirect to profile or home
  if (isAuthenticated) {
    // In a real implementation, you might want to use a router to redirect
    if (typeof window !== 'undefined') {
      window.location.href = '/profile';
    }
    return <div>Redirecting...</div>;
  }

  return (
    <Layout title="Login" description="Login to your account">
      <div style={{ padding: '2rem', maxWidth: '400px', margin: '2rem auto' }}>
        <h1>Login</h1>
        <LoginForm />
      </div>
    </Layout>
  );
}

function LoginForm() {
  const { login } = useAuth();
  const [email, setEmail] = React.useState('');
  const [password, setPassword] = React.useState('');
  const [error, setError] = React.useState('');
  const [loading, setLoading] = React.useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await login(email, password);
      if (!result.success) {
        setError(result.message);
      }
    } catch (err) {
      setError('An error occurred during login');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="email" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Email
        </label>
        <input
          id="email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
          style={{
            width: '100%',
            padding: '0.5rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
          }}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="password" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Password
        </label>
        <input
          id="password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
          style={{
            width: '100%',
            padding: '0.5rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
          }}
        />
      </div>
      {error && (
        <div style={{ color: 'red', marginBottom: '1rem' }}>{error}</div>
      )}
      <button
        type="submit"
        disabled={loading}
        style={{
          width: '100%',
          padding: '0.75rem',
          backgroundColor: loading ? '#ccc' : '#007cba',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: loading ? 'not-allowed' : 'pointer',
        }}
      >
        {loading ? 'Logging in...' : 'Login'}
      </button>
      <div style={{ marginTop: '1rem', textAlign: 'center' }}>
        <p>
          Don't have an account? <a href="/register">Register here</a>
        </p>
      </div>
    </form>
  );
}