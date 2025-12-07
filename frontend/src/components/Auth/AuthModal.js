import React from "react";
import Login from "./Login";
import Register from "./Register";
import BrowserOnly from "@docusaurus/BrowserOnly";

const AuthModalContent = ({ isOpen, onClose }) => {
  const [view, setView] = React.useState("login"); // 'login' or 'register'

  if (!isOpen) return null;

  return (
    <div className="auth-modal-overlay">
      <div className="auth-modal-container">
        <button className="auth-modal-close" onClick={onClose}>
          ×
        </button>

        {view === "login" ? (
          <Login
            onSwitchToRegister={() => setView("register")}
            onClose={onClose}
          />
        ) : (
          <Register
            onSwitchToLogin={() => setView("login")}
            onClose={onClose}
          />
        )}
      </div>
    </div>
  );
};

const AuthModal = (props) => {
  return (
    <BrowserOnly fallback={null}>
      {() => <AuthModalContent {...props} />}
    </BrowserOnly>
  );
};

export default AuthModal;
