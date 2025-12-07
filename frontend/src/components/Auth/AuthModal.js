import React, { useState } from 'react';
import Login from './Login';
import Register from './Register';

const AuthModal = ({ isOpen, onClose }) => {
  const [isLoginView, setIsLoginView] = useState(true);

  if (!isOpen) return null;

  return (
    <div className="auth-modal-wrapper">
      <div className="auth-modal-overlay" onClick={onClose} />
      <div className="auth-modal-content">
        {isLoginView ? (
          <Login
            onSwitchToRegister={() => setIsLoginView(false)}
            onClose={onClose}
          />
        ) : (
          <Register
            onSwitchToLogin={() => setIsLoginView(true)}
            onClose={onClose}
          />
        )}
      </div>
      <style jsx>{`
        .auth-modal-wrapper {
          position: fixed;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          display: flex;
          align-items: center;
          justify-content: center;
          z-index: 1000;
        }

        .auth-modal-overlay {
          position: absolute;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          background-color: rgba(0, 0, 0, 0.5);
        }

        .auth-modal-content {
          position: relative;
          z-index: 1001;
        }
      `}</style>
    </div>
  );
};

export default AuthModal;