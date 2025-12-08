import React, { useState } from 'react';
import { useAuth } from './AuthProvider';

const Login = ({ onSwitchToRegister, onClose }) => {
  const { login } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    const result = await login(email, password);
    
    if (result.success) {
      onClose();
      window.location.reload();
    } else {
      setError(result.message);
    }
    setLoading(false);
  };

  return (
    <div className="auth-form-container">
      <h2>Welcome Back</h2>
      {error && <div className="auth-error">{error}</div>}
      
      <form onSubmit={handleSubmit}>
        <div className="auth-input-group">
          <label>Email Address</label>
          <input 
            type="email" 
            value={email} 
            onChange={(e) => setEmail(e.target.value)}
            required 
            placeholder="name@example.com"
          />
        </div>
        
        <div className="auth-input-group">
          <label>Password</label>
          <input 
            type="password" 
            value={password} 
            onChange={(e) => setPassword(e.target.value)}
            required 
            placeholder="••••••••"
          />
        </div>

        <button type="submit" className="auth-submit-btn" disabled={loading}>
          {loading ? 'Signing in...' : 'Sign In'}
        </button>
      </form>

      <div className="auth-footer">
        <p>
          New here?{' '}
          <button className="auth-link-btn" onClick={onSwitchToRegister}>
            Create an account
          </button>
        </p>
      </div>

      {/* 🟢 EMBEDDED STYLES */}
      <style>{`
        .auth-form-container h2 {
          margin-top: 0;
          margin-bottom: 20px;
          color: #ffffff;
          text-align: center;
          font-size: 24px;
        }
        .auth-error {
          background-color: rgba(239, 68, 68, 0.2);
          border: 1px solid #ef4444;
          color: #fca5a5;
          padding: 10px;
          border-radius: 8px;
          margin-bottom: 1rem;
          font-size: 14px;
          text-align: center;
        }
        .auth-input-group {
          margin-bottom: 1.2rem;
        }
        .auth-input-group label {
          display: block;
          margin-bottom: 8px;
          color: #94a3b8;
          font-size: 14px;
          font-weight: 500;
        }
        .auth-input-group input {
          width: 100%;
          padding: 12px;
          border-radius: 8px;
          border: 1px solid #334155;
          background-color: #0f172a;
          color: white;
          font-size: 16px;
          transition: border-color 0.2s;
        }
        .auth-input-group input:focus {
          border-color: #2ECC71;
          outline: none;
        }
        .auth-submit-btn {
          width: 100%;
          padding: 12px;
          background-color: #2ECC71;
          color: #0f172a;
          border: none;
          border-radius: 8px;
          font-weight: bold;
          font-size: 16px;
          cursor: pointer;
          margin-top: 10px;
          transition: transform 0.1s, background-color 0.2s;
        }
        .auth-submit-btn:hover {
          background-color: #22c55e;
        }
        .auth-submit-btn:active {
          transform: scale(0.98);
        }
        .auth-submit-btn:disabled {
          opacity: 0.7;
          cursor: not-allowed;
        }
        .auth-footer {
          margin-top: 2rem;
          text-align: center;
          font-size: 14px;
          color: #94a3b8;
        }
        .auth-link-btn {
          background: none;
          border: none;
          color: #2ECC71;
          cursor: pointer;
          font-weight: bold;
          padding: 0 4px;
        }
        .auth-link-btn:hover {
          text-decoration: underline;
        }
      `}</style>
    </div>
  );
};

export default Login;