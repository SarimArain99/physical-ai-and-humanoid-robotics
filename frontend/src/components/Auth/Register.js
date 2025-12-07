import React, { useState } from "react";
import { useAuth } from "./AuthProvider";

const Register = ({ onSwitchToLogin, onClose }) => {
  const { register } = useAuth();
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [proficiency, setProficiency] = useState("beginner"); // Default
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    // Pass proficiency to the updated register function
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
          />
        </div>

        <div className="auth-input-group">
          <label>Email</label>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
        </div>

        <div className="auth-input-group">
          <label>Password</label>
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
        </div>

        {/* Proficiency Dropdown */}
        <div className="auth-input-group">
          <label htmlFor="proficiency">Proficiency Level</label>
          <select
            id="proficiency"
            value={proficiency}
            onChange={(e) => setProficiency(e.target.value)}
            disabled={loading}
            className="auth-select"
          >
            <option value="beginner">Beginner (New to AI/Robotics)</option>
            <option value="intermediate">
              Intermediate (Engineering Student)
            </option>
            <option value="pro">Pro (Researcher/Developer)</option>
          </select>
        </div>

        <button type="submit" className="auth-submit-btn" disabled={loading}>
          {loading ? "Creating Account..." : "Register"}
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
    </div>
  );
};

export default Register;
