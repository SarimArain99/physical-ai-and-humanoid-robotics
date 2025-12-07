import React, { useState } from "react";
import { useAuth } from "./AuthProvider";
import AuthModal from "./AuthModal";
import BrowserOnly from "@docusaurus/BrowserOnly"; // 🟢 Import

// 1. Logic Component
const AuthButtonContent = () => {
  const { user, logout, loading } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleLogout = async () => {
    await logout();
    window.location.reload();
  };

  if (loading) {
    return <div style={{ fontSize: "14px", color: "gray" }}>...</div>;
  }

  const getDisplayName = () => {
    if (!user) return "";
    if (user.name && user.name.trim() !== "") return user.name;
    if (user.email) return user.email.split("@")[0];
    return "User";
  };

  return (
    <>
      {user ? (
        <div className="auth-dropdown">
          <button className="auth-user-button">
            <span className="auth-user-name">{getDisplayName()}</span>
            <span style={{ fontSize: "10px", marginLeft: "6px", opacity: 0.7 }}>
              ▼
            </span>
          </button>
          <div className="auth-dropdown-content">
            <div className="auth-user-email-display">{user.email}</div>
            <a href="/profile" className="auth-dropdown-item">
              Profile
            </a>
            <button className="auth-dropdown-item" onClick={handleLogout}>
              Logout
            </button>
          </div>
        </div>
      ) : (
        <button className="auth-button" onClick={() => setIsModalOpen(true)}>
          Sign In
        </button>
      )}
      <AuthModal isOpen={isModalOpen} onClose={() => setIsModalOpen(false)} />
    </>
  );
};

// 2. Safe Wrapper
const AuthButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <AuthButtonContent />}</BrowserOnly>
  );
};

export default AuthButton;
