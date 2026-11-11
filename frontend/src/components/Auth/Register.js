import React, { useState } from "react";
import { useAuth } from "./AuthProvider";

const Register = ({ onSwitchToLogin, onClose }) => {
  const { register } = useAuth();
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [proficiency, setProficiency] = useState("beginner");
  const [softwareBg, setSoftwareBg] = useState("");
  const [hardwareBg, setHardwareBg] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    const result = await register(email, name, password, {
      proficiency,
      softwareBg,
      hardwareBg,
    });

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
      <h2>Create Account</h2>
      {error && <div className="auth-error">{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className="auth-input-group">
          <label htmlFor="register-name">Full Name</label>
          <input
            id="register-name"
            type="text"
            autoComplete="name"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            placeholder="John Doe"
          />
        </div>

        <div className="auth-input-group">
          <label htmlFor="register-email">Email</label>
          <input
            id="register-email"
            type="email"
            autoComplete="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            placeholder="name@example.com"
          />
        </div>

        <div className="auth-input-group">
          <label htmlFor="register-password">Password</label>
          <input
            id="register-password"
            type="password"
            autoComplete="new-password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            minLength={8}
            placeholder="••••••••"
          />
        </div>

        <div className="auth-input-group">
          <label htmlFor="register-proficiency">Proficiency Level</label>
          <select
            id="register-proficiency"
            value={proficiency}
            onChange={(e) => setProficiency(e.target.value)}
            className="auth-select"
          >
            <option value="beginner">Beginner (New to AI)</option>
            <option value="intermediate">Intermediate (Student)</option>
            <option value="pro">Pro (Researcher)</option>
          </select>
        </div>

        <div className="auth-section">
          <h3>Your background <span className="auth-optional">Optional</span></h3>
          <p className="auth-section-note">So we can personalise the chapters for you.</p>

          <div className="auth-input-group">
            <label htmlFor="register-software-bg">Software background</label>
            <input
              id="register-software-bg"
              type="text"
              autoComplete="off"
              value={softwareBg}
              onChange={(e) => setSoftwareBg(e.target.value)}
              maxLength={200}
              placeholder="e.g. Python, or new to coding"
            />
          </div>

          <div className="auth-input-group">
            <label htmlFor="register-hardware-bg">Hardware access</label>
            <input
              id="register-hardware-bg"
              type="text"
              autoComplete="off"
              value={hardwareBg}
              onChange={(e) => setHardwareBg(e.target.value)}
              maxLength={200}
              placeholder="e.g. Laptop, or no hardware yet"
            />
          </div>
        </div>

        <button type="submit" className="auth-submit-btn" disabled={loading}>
          {loading ? "Creating..." : "Register"}
        </button>
      </form>

      <div className="auth-footer">
        <p>
          Already have an account?{" "}
          <button className="auth-link-btn" onClick={onSwitchToLogin}>
            Sign In
          </button>
        </p>
      </div>

      {/* Embedded styles (same styles as Login, plus the select style) */}
      <style>{`
        .auth-form-container h2 { margin-top: 0; margin-bottom: 16px; color: #fff; text-align: center; }
        .auth-error { background: rgba(239,68,68,0.2); border: 1px solid #ef4444; color: #fca5a5; padding: 10px; border-radius: 8px; margin-bottom: 1rem; text-align: center; font-size: 14px; }
        .auth-input-group { margin-bottom: 1rem; }
        .auth-input-group label { display: block; margin-bottom: 8px; color: #94a3b8; font-size: 14px; font-weight: 500; }
        .auth-input-group input, .auth-select { width: 100%; padding: 12px; border-radius: 8px; border: 1px solid #334155; background-color: #0f172a; color: white; font-size: 16px; }
        .auth-input-group input:focus, .auth-select:focus, .auth-input-group input:focus-visible, .auth-select:focus-visible { border-color: #58A6FF; outline: none; }
        .auth-section { margin-top: 4px; padding-top: 12px; border-top: 1px solid #334155; }
        .auth-section h3 { margin: 0 0 4px; color: #fff; font-size: 16px; font-weight: 600; }
        .auth-section-note { margin: 0 0 12px; color: #94a3b8; font-size: 14px; }
        .auth-optional { color: #94a3b8; font-size: 12px; font-weight: 500; }
        .auth-submit-btn { width: 100%; padding: 12px; background-color: #58A6FF; color: #0f172a; border: none; border-radius: 8px; font-weight: bold; font-size: 16px; cursor: pointer; margin-top: 8px; }
        .auth-submit-btn:hover { background-color: #388BFD; }
        .auth-submit-btn:active { transform: scale(0.98); }
        .auth-submit-btn:disabled { opacity: 0.7; }
        .auth-submit-btn:focus-visible, .auth-link-btn:focus-visible { outline: 2px solid #58A6FF; outline-offset: 2px; }
        .auth-footer { margin-top: 1.2rem; text-align: center; font-size: 14px; color: #94a3b8; }
        .auth-link-btn { background: none; border: none; color: #58A6FF; cursor: pointer; font-weight: bold; padding: 0 4px; }
      `}</style>
    </div>
  );
};

export default Register;
