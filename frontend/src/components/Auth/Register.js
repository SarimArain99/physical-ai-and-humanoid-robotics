import React, { useState } from "react";
import { useAuth } from "./AuthProvider";

const Register = ({ onSwitchToLogin, onClose }) => {
  const { register } = useAuth();
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [proficiency, setProficiency] = useState("beginner");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    const result = await register(email, name, password, proficiency);

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
          <label>Full Name</label>
          <input
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            placeholder="John Doe"
          />
        </div>

        <div className="auth-input-group">
          <label>Email</label>
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

        <div className="auth-input-group">
          <label>Proficiency Level</label>
          <select
            value={proficiency}
            onChange={(e) => setProficiency(e.target.value)}
            className="auth-select"
          >
            <option value="beginner">Beginner (New to AI)</option>
            <option value="intermediate">Intermediate (Student)</option>
            <option value="pro">Pro (Researcher)</option>
          </select>
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

      {/* 🟢 EMBEDDED STYLES (Reusing same styles as Login + Select style) */}
      <style>{`
        .auth-form-container h2 { margin-top: 0; margin-bottom: 20px; color: #fff; text-align: center; }
        .auth-error { background: rgba(239,68,68,0.2); border: 1px solid #ef4444; color: #fca5a5; padding: 10px; border-radius: 8px; margin-bottom: 1rem; text-align: center; font-size: 14px; }
        .auth-input-group { margin-bottom: 1.2rem; }
        .auth-input-group label { display: block; margin-bottom: 8px; color: #94a3b8; font-size: 14px; font-weight: 500; }
        .auth-input-group input, .auth-select { width: 100%; padding: 12px; border-radius: 8px; border: 1px solid #334155; background-color: #0f172a; color: white; font-size: 16px; }
        .auth-input-group input:focus, .auth-select:focus { border-color: #2ECC71; outline: none; }
        .auth-submit-btn { width: 100%; padding: 12px; background-color: #2ECC71; color: #0f172a; border: none; border-radius: 8px; font-weight: bold; font-size: 16px; cursor: pointer; margin-top: 10px; }
        .auth-submit-btn:hover { background-color: #22c55e; }
        .auth-submit-btn:disabled { opacity: 0.7; }
        .auth-footer { margin-top: 2rem; text-align: center; font-size: 14px; color: #94a3b8; }
        .auth-link-btn { background: none; border: none; color: #2ECC71; cursor: pointer; font-weight: bold; padding: 0 4px; }
      `}</style>
    </div>
  );
};

export default Register;
