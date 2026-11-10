/**
 * Auth Button Component
 * Feature: 002-ui-improvements
 * Task: Unified navbar button system
 *
 * Sign In / User Profile dropdown with unified styling.
 */
import React, { useState, useEffect, useRef } from "react";
import { useAuth } from "./AuthProvider";
import AuthModal from "./AuthModal";
import BrowserOnly from "@docusaurus/BrowserOnly";
import "./AuthButton.css";

const UserIcon = () => (
  <svg
    width="18"
    height="18"
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
    width="18"
    height="18"
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

const AuthButtonContent = ({ onClick: onNavbarItemClick }) => {
  const { user, logout, loading } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef(null);

  // Docusaurus passes onClick to every navbar item in the mobile
  // sidebar (Navbar/MobileSidebar/PrimaryMenu): it toggles the sidebar
  // closed. Desktop navbar items get no onClick. Calling it here is
  // what closes the hamburger cleanly when the modal opens — the modal
  // itself is portaled to document.body, so it stays open and centred
  // on the viewport after the sidebar is gone. Before this, the prop
  // was silently dropped, so the sidebar stayed open behind the modal.
  const openModal = () => {
    setIsModalOpen(true);
    if (onNavbarItemClick) onNavbarItemClick();
  };

  // The menu opens on click, not on hover: hover menus never open on
  // touch screens and keyboard users cannot reach them at all.
  useEffect(() => {
    if (!isOpen) return;
    const onKeyDown = (event) => {
      if (event.key === "Escape") setIsOpen(false);
    };
    const onPointerDown = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };
    document.addEventListener("keydown", onKeyDown);
    document.addEventListener("mousedown", onPointerDown);
    return () => {
      document.removeEventListener("keydown", onKeyDown);
      document.removeEventListener("mousedown", onPointerDown);
    };
  }, [isOpen]);

  const handleLogout = async () => {
    await logout();
    window.location.reload();
  };

  // While the session is being checked the bar keeps a 36px placeholder
  // pill, so the controls do not jump when the real button appears.
  if (loading)
    return (
      <div className="auth-button-wrapper">
        <div
          className="navbar-btn navbar-btn--secondary navbar-btn--loading auth-loading-pill"
          aria-hidden="true"
        />
      </div>
    );

  const getDisplayName = () => {
    if (!user) return "";
    return user.name || user.email.split("@")[0];
  };

  return (
    <div className="auth-button-wrapper">
      {user ? (
        <div
          className={`auth-dropdown ${isOpen ? "auth-dropdown--open" : ""}`}
          ref={dropdownRef}
        >
          <button
            className="auth-user-trigger navbar-btn navbar-btn--secondary"
            onClick={() => setIsOpen(!isOpen)}
            aria-expanded={isOpen}
            aria-haspopup="menu"
          >
            <span className="auth-user-icon"><UserIcon /></span>
            <span className="auth-user-name navbar-btn__label">{getDisplayName()}</span>
            <span className="auth-chevron"><ChevronDownIcon /></span>
          </button>

          {isOpen && (
            <div className="auth-dropdown-menu" role="menu">
              <div className="auth-dropdown-header">{user.email}</div>

              <a href="/profile" className="auth-dropdown-item" role="menuitem">
                <span className="auth-dropdown-icon"><UserIcon /></span>
                My Profile
              </a>

              <div className="auth-dropdown-divider"></div>

              <button
                className="auth-dropdown-item auth-dropdown-item--logout"
                onClick={handleLogout}
                role="menuitem"
              >
                <span className="auth-dropdown-icon"><LogoutIcon /></span>
                Logout
              </button>
            </div>
          )}
        </div>
      ) : (
        <button
          className="navbar-btn navbar-btn--primary"
          onClick={openModal}
        >
          Sign In
        </button>
      )}

      <AuthModal isOpen={isModalOpen} onClose={() => setIsModalOpen(false)} />
    </div>
  );
};

const AuthButton = (props) => {
  return (
    <BrowserOnly fallback={null}>
      {() => <AuthButtonContent {...props} />}
    </BrowserOnly>
  );
};

export default AuthButton;
