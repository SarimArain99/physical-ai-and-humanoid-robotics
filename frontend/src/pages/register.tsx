import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthProvider';

export default function Register() {
  const { isAuthenticated } = useAuth();

  // If already authenticated, redirect to profile or home
  if (isAuthenticated) {
    // In a real implementation, you might want to use a router to redirect
    if (typeof window !== 'undefined') {
      window.location.href = '/profile';
    }
    return <div>Redirecting…</div>;
  }

  return (
    <Layout title="Register" description="Create a new account">
      <div style={{ padding: '2rem', maxWidth: '400px', margin: '2rem auto' }}>
        <h1>Register</h1>
        <RegisterForm />
      </div>
    </Layout>
  );
}

function RegisterForm() {
  const { register } = useAuth();
  const [email, setEmail] = React.useState('');
  const [name, setName] = React.useState('');
  const [password, setPassword] = React.useState('');
  const [confirmPassword, setConfirmPassword] = React.useState('');
  const [proficiency, setProficiency] = React.useState('beginner');
  const [softwareBg, setSoftwareBg] = React.useState('');
  const [hardwareBg, setHardwareBg] = React.useState('');
  const [error, setError] = React.useState('');
  const [loading, setLoading] = React.useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      setLoading(false);
      return;
    }

    try {
      const result = await register(email, name, password, {
        proficiency,
        softwareBg,
        hardwareBg,
      });
      if (!result.success) {
        setError(result.message);
      } else {
        // Registration successful, redirect to login
        if (typeof window !== 'undefined') {
          window.location.href = '/login';
        }
      }
    } catch (err) {
      setError('An error occurred during registration');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="name" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Name
        </label>
        <input
          id="name"
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
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
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="confirmPassword" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Confirm Password
        </label>
        <input
          id="confirmPassword"
          type="password"
          value={confirmPassword}
          onChange={(e) => setConfirmPassword(e.target.value)}
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
        <label htmlFor="proficiency" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Proficiency Level
        </label>
        <select
          id="proficiency"
          value={proficiency}
          onChange={(e) => setProficiency(e.target.value)}
          style={{
            width: '100%',
            padding: '0.5rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
          }}
        >
          <option value="beginner">Beginner (New to AI)</option>
          <option value="intermediate">Intermediate (Student)</option>
          <option value="pro">Pro (Researcher)</option>
        </select>
      </div>
      <p style={{ margin: '0 0 1rem', fontSize: '14px' }}>
        Your background (optional) — so the chapters can be personalised for you.
      </p>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="softwareBg" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Software background
        </label>
        <input
          id="softwareBg"
          type="text"
          value={softwareBg}
          onChange={(e) => setSoftwareBg(e.target.value)}
          maxLength={200}
          placeholder="e.g. Python, or new to coding"
          style={{
            width: '100%',
            padding: '0.5rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
          }}
        />
      </div>
      <div style={{ marginBottom: '1rem' }}>
        <label htmlFor="hardwareBg" style={{ display: 'block', marginBottom: '0.5rem' }}>
          Hardware access
        </label>
        <input
          id="hardwareBg"
          type="text"
          value={hardwareBg}
          onChange={(e) => setHardwareBg(e.target.value)}
          maxLength={200}
          placeholder="e.g. Laptop, or no hardware yet"
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
        {loading ? 'Registering…' : 'Register'}
      </button>
      <div style={{ marginTop: '1rem', textAlign: 'center' }}>
        <p>
          Already have an account? <a href="/login">Login here</a>
        </p>
      </div>
    </form>
  );
}