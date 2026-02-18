/**
 * Auth Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Sign In / User Profile dropdown with unified styling.
 */
import React, { useState } from "react";
import { useAuth } from "./AuthProvider";
import AuthModal from "./AuthModal";
import BrowserOnly from "@docusaurus/BrowserOnly";
import "./AuthButton.css";

const AuthButtonContent = () => {
  const { user, logout, loading } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleLogout = async () => {
    await logout();
    window.location.reload();
  };

  if (loading)
    return (
      <div className="auth-button-wrapper">
        <div className="navbar-loading">...</div>
      </div>
    );

  const getDisplayName = () => {
    if (!user) return "";
    return user.name || user.email.split("@")[0];
  };

  return (
    <div className="auth-button-wrapper">
      {user ? (
        <div className="auth-dropdown">
          <button className="auth-user-trigger navbar-btn navbar-btn--secondary">
            <span className="auth-user-icon">👤</span>
            <span className="auth-user-name navbar-btn__label">{getDisplayName()}</span>
            <span className="auth-chevron">▼</span>
          </button>

          <div className="auth-dropdown-menu">
            <div className="auth-dropdown-header">{user.email}</div>

            <a href="/profile" className="auth-dropdown-item">
              <span className="auth-dropdown-icon">👤</span>
              My Profile
            </a>

            <div className="auth-dropdown-divider"></div>

            <button
              className="auth-dropdown-item auth-dropdown-item--logout"
              onClick={handleLogout}
            >
              <span className="auth-dropdown-icon">🚪</span>
              Logout
            </button>
          </div>
        </div>
      ) : (
        <button
          className="navbar-btn navbar-btn--primary auth-signin-btn"
          onClick={() => setIsModalOpen(true)}
        >
          Sign In
        </button>
      )}

      <AuthModal isOpen={isModalOpen} onClose={() => setIsModalOpen(false)} />
    </div>
  );
};

const AuthButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <AuthButtonContent />}</BrowserOnly>
  );
};

export default AuthButton;
