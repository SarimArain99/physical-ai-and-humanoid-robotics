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

const UserIcon = () => (
  <svg
    width="16"
    height="16"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
    <circle cx="12" cy="7" r="4"></circle>
  </svg>
);

const LogoutIcon = () => (
  <svg
    width="16"
    height="16"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"></path>
    <polyline points="16 17 21 12 16 7"></polyline>
    <line x1="21" y1="12" x2="9" y2="12"></line>
  </svg>
);

const ChevronDownIcon = () => (
  <svg
    width="12"
    height="12"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2.5"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <path d="M6 9l6 6 6-6" />
  </svg>
);

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
            <span className="auth-user-icon"><UserIcon /></span>
            <span className="auth-user-name navbar-btn__label">{getDisplayName()}</span>
            <span className="auth-chevron"><ChevronDownIcon /></span>
          </button>

          <div className="auth-dropdown-menu">
            <div className="auth-dropdown-header">{user.email}</div>

            <a href="/profile" className="auth-dropdown-item">
              <span className="auth-dropdown-icon"><UserIcon /></span>
              My Profile
            </a>

            <div className="auth-dropdown-divider"></div>

            <button
              className="auth-dropdown-item auth-dropdown-item--logout"
              onClick={handleLogout}
            >
              <span className="auth-dropdown-icon"><LogoutIcon /></span>
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
