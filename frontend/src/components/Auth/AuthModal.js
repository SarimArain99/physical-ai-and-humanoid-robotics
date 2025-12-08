import React from 'react';
import Login from './Login';
import Register from './Register';
import BrowserOnly from '@docusaurus/BrowserOnly';

const AuthModalContent = ({ isOpen, onClose }) => {
  const [view, setView] = React.useState('login');

  if (!isOpen) return null;

  return (
    <div className="auth-modal-overlay">
      <div className="auth-modal-container">
        <button className="auth-modal-close" onClick={onClose}>×</button>
        
        {view === 'login' ? (
          <Login 
            onSwitchToRegister={() => setView('register')} 
            onClose={onClose} 
          />
        ) : (
          <Register 
            onSwitchToLogin={() => setView('login')} 
            onClose={onClose} 
          />
        )}
      </div>

      {/* 🟢 EMBEDDED STYLES */}
      <style>{`
        .auth-modal-overlay {
          position: fixed;
          top: 0;
          left: 0;
          width: 100vw;
          height: 100vh;
          background-color: rgba(0, 0, 0, 0.85);
          display: flex;
          justify-content: center;
          align-items: center;
          z-index: 99999;
          backdrop-filter: blur(5px);
        }
        .auth-modal-container {
          background-color: #1E2A38;
          padding: 2.5rem;
          border-radius: 16px;
          width: 90%;
          max-width: 420px;
          position: relative;
          box-shadow: 0 20px 50px rgba(0,0,0,0.5);
          border: 1px solid #2ECC71;
          color: white;
          animation: fadeIn 0.3s ease-out;
        }
        .auth-modal-close {
          position: absolute;
          top: 15px;
          right: 20px;
          background: none;
          border: none;
          font-size: 28px;
          color: #94a3b8;
          cursor: pointer;
          transition: color 0.2s;
        }
        .auth-modal-close:hover {
          color: #ffffff;
        }
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(-20px); }
          to { opacity: 1; transform: translateY(0); }
        }
      `}</style>
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