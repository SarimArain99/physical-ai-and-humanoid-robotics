import React, { useState } from "react";
import { useAuth } from "./AuthProvider";
import AuthModal from "./AuthModal";
import BrowserOnly from "@docusaurus/BrowserOnly";

const AuthButtonContent = () => {
  const { user, logout, loading } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleLogout = async () => {
    await logout();
    window.location.reload();
  };

  if (loading)
    return <div style={{ color: "#2ECC71", fontSize: "14px" }}>...</div>;

  const getDisplayName = () => {
    if (!user) return "";
    return user.name || user.email.split("@")[0];
  };

  return (
    <>
      {user ? (
        <div className="auth-dropdown-wrapper">
          <button className="auth-user-trigger">
            <span className="user-icon">👤</span>
            <span className="user-name">{getDisplayName()}</span>
          </button>

          <div className="auth-dropdown-menu">
            <div className="auth-dropdown-header">{user.email}</div>
            <div className="auth-dropdown-divider"></div>
            <button className="auth-dropdown-item" onClick={handleLogout}>
              Logout
            </button>
          </div>
        </div>
      ) : (
        <button className="auth-login-btn" onClick={() => setIsModalOpen(true)}>
          Sign In
        </button>
      )}

      <AuthModal isOpen={isModalOpen} onClose={() => setIsModalOpen(false)} />

      {/* 🟢 EMBEDDED STYLES */}
      <style>{`
        /* Login Button */
        .auth-login-btn {
          background-color: #2ECC71;
          color: #1E2A38;
          border: none;
          padding: 8px 20px;
          border-radius: 20px;
          font-weight: bold;
          cursor: pointer;
          font-size: 14px;
          transition: transform 0.2s;
          margin-left: 10px;
        }
        .auth-login-btn:hover {
          transform: scale(1.05);
          background-color: #22c55e;
        }

        /* User Dropdown Trigger */
        .auth-dropdown-wrapper {
          position: relative;
          margin-left: 10px;
          display: inline-block;
        }
        .auth-user-trigger {
          background: transparent;
          border: 1px solid #2ECC71;
          color: #2ECC71;
          padding: 6px 16px;
          border-radius: 20px;
          cursor: pointer;
          display: flex;
          align-items: center;
          gap: 8px;
          font-size: 14px;
          font-weight: 500;
        }
        .auth-user-trigger:hover {
          background: rgba(46, 204, 113, 0.1);
        }

        /* Dropdown Menu */
        .auth-dropdown-menu {
          display: none;
          position: absolute;
          right: 0;
          top: 100%;
          margin-top: 10px;
          background-color: #1E2A38;
          min-width: 200px;
          border-radius: 12px;
          border: 1px solid #334155;
          box-shadow: 0 10px 30px rgba(0,0,0,0.5);
          z-index: 1000;
          overflow: hidden;
        }
        .auth-dropdown-wrapper:hover .auth-dropdown-menu {
          display: block;
          animation: slideDown 0.2s ease-out;
        }
        .auth-dropdown-header {
          padding: 12px 16px;
          font-size: 12px;
          color: #94a3b8;
          background-color: #151e29;
        }
        .auth-dropdown-item {
          display: block;
          width: 100%;
          text-align: left;
          padding: 12px 16px;
          background: none;
          border: none;
          color: white;
          cursor: pointer;
          font-size: 14px;
        }
        .auth-dropdown-item:hover {
          background-color: #2ECC71;
          color: #1E2A38;
        }
        @keyframes slideDown {
          from { opacity: 0; transform: translateY(-10px); }
          to { opacity: 1; transform: translateY(0); }
        }
      `}</style>
    </>
  );
};

const AuthButton = () => {
  return (
    <BrowserOnly fallback={null}>{() => <AuthButtonContent />}</BrowserOnly>
  );
};

export default AuthButton;
