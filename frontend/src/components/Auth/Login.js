import React, { useState, useEffect } from "react";
import { useAuth } from "./AuthProvider";
import { useHistory } from "@docusaurus/router"; // ✅ Added for redirection

const Login = ({ onSwitchToRegister, onClose }) => {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  // ✅ Get 'user' to check login status
  const { login, user } = useAuth();
  const history = useHistory(); // ✅ Initialize history

  // 🟢 AUTOMATIC REDIRECT LOGIC
  // If the user is already logged in, close the modal and go to profile
  useEffect(() => {
    if (user) {
      if (onClose) onClose(); // Close the modal if it's open
      history.push("/profile"); // Redirect to profile page
    }
  }, [user, history, onClose]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    const result = await login(email, password);

    if (result.success) {
      // The useEffect above will handle the redirect
      // checking 'user' state change
      if (onClose) onClose();
    } else {
      setError(result.message || "Login failed");
    }

    setLoading(false);
  };

  return (
    <div className="auth-modal">
      <div className="auth-form">
        <h2>Login</h2>
        {error && <div className="auth-error">{error}</div>}
        <form onSubmit={handleSubmit}>
          <div className="auth-input-group">
            <label htmlFor="email">Email</label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              disabled={loading}
            />
          </div>
          <div className="auth-input-group">
            <label htmlFor="password">Password</label>
            <input
              id="password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              disabled={loading}
            />
          </div>
          <button type="submit" disabled={loading} className="auth-button">
            {loading ? "Logging in..." : "Login"}
          </button>
        </form>
        <div className="auth-switch">
          Don't have an account?{" "}
          <button
            type="button"
            onClick={onSwitchToRegister}
            disabled={loading}
            className="auth-link"
          >
            Register
          </button>
        </div>
      </div>
      <style jsx>{`
        .auth-modal {
          position: fixed;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          background: rgba(0, 0, 0, 0.7);
          display: flex;
          align-items: center;
          justify-content: center;
          z-index: 1000;
        }

        .auth-form {
          background: #1f2937;
          padding: 2rem;
          border-radius: 8px;
          width: 100%;
          max-width: 400px;
          box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }

        .auth-form h2 {
          margin-top: 0;
          margin-bottom: 1.5rem;
          text-align: center;
          color: #f9fafb;
        }

        .auth-input-group {
          margin-bottom: 1rem;
        }

        .auth-input-group label {
          display: block;
          margin-bottom: 0.5rem;
          font-weight: 500;
          color: #e5e7eb;
        }

        .auth-input-group input {
          width: 100%;
          padding: 0.75rem;
          border: 1px solid #374151;
          border-radius: 4px;
          font-size: 1rem;
          background-color: #111827;
          color: #f9fafb;
        }

        .auth-input-group input:focus {
          outline: none;
          border-color: #3b82f6;
          box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.2);
        }

        .auth-button {
          width: 100%;
          padding: 0.75rem;
          background-color: #3b82f6;
          color: white;
          border: none;
          border-radius: 4px;
          font-size: 1rem;
          cursor: pointer;
          margin-bottom: 1rem;
        }

        .auth-button:hover:not(:disabled) {
          background-color: #2563eb;
        }

        .auth-button:disabled {
          background-color: #6b7280;
          cursor: not-allowed;
        }

        .auth-error {
          color: #f87171;
          padding: 0.5rem;
          background-color: #3f3f46;
          border: 1px solid #7f1d1d;
          border-radius: 4px;
          margin-bottom: 1rem;
        }

        .auth-switch {
          text-align: center;
          color: #e5e7eb;
        }

        .auth-link {
          background: none;
          border: none;
          color: #60a5fa;
          cursor: pointer;
          text-decoration: underline;
        }

        .auth-link:disabled {
          color: #9ca3af;
          cursor: not-allowed;
        }
      `}</style>
    </div>
  );
};

export default Login;
