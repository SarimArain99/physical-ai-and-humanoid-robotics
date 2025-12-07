import React, { useState, useEffect } from "react";
import { useAuth } from "./AuthProvider";
import AuthModal from "./AuthModal";
import ExecutionEnvironment from "@docusaurus/ExecutionEnvironment";

const AuthButton = () => {
  // 🛡️ SAFETY CHECK: If running on server during build, render nothing
  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  const { user, logout, loading } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  // Debugging: Watch the console to see if user data arrives
  useEffect(() => {
    console.log("AuthButton User State:", user);
  }, [user]);

  const handleLogout = async () => {
    await logout();
    window.location.reload(); // Refresh to clear state completely
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
        // 🟢 LOGGED IN VIEW
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
        // 🔴 LOGGED OUT VIEW
        <button className="auth-button" onClick={() => setIsModalOpen(true)}>
          Sign In
        </button>
      )}

      <AuthModal isOpen={isModalOpen} onClose={() => setIsModalOpen(false)} />
    </>
  );
};

export default AuthButton;
