import React from 'react';
import { createPortal } from 'react-dom';
import Login from './Login';
import Register from './Register';
import BrowserOnly from '@docusaurus/BrowserOnly';

const AuthModalContent = ({ isOpen, onClose }) => {
  const [view, setView] = React.useState('login');
  const closeButtonRef = React.useRef(null);
  const previouslyFocusedRef = React.useRef(null);
  // onClose is an inline arrow at the call sites, so its identity
  // changes every render. Keep it in a ref so the effect below runs
  // only when the modal opens or closes, never mid-typing.
  const onCloseRef = React.useRef(onClose);
  onCloseRef.current = onClose;

  // A fresh open always starts on sign-in; the view never leaks from a
  // previous open (e.g. register) into the next one.
  React.useEffect(() => {
    if (isOpen) {
      setView('login');
    }
  }, [isOpen]);

  // Close on Escape, and lock the page scroll behind the modal. The
  // scroll lock is restored exactly, not blanked, so a page that had
  // its own overflow value keeps it.
  React.useEffect(() => {
    if (!isOpen) return undefined;
    previouslyFocusedRef.current = document.activeElement;
    const previousOverflow = document.body.style.overflow;
    document.body.style.overflow = 'hidden';
    const onKeyDown = (event) => {
      if (event.key === 'Escape') onCloseRef.current();
    };
    document.addEventListener('keydown', onKeyDown);
    if (closeButtonRef.current) closeButtonRef.current.focus();
    return () => {
      document.body.style.overflow = previousOverflow;
      document.removeEventListener('keydown', onKeyDown);
      if (
        previouslyFocusedRef.current &&
        typeof previouslyFocusedRef.current.focus === 'function'
      ) {
        previouslyFocusedRef.current.focus();
      }
    };
  }, [isOpen]);

  if (!isOpen) return null;

  // Portaled to document.body: the modal must never live inside the
  // navbar or the mobile sidebar. The open sidebar carries a transform
  // (translateZ(0) when shown) and overflow-x hidden, which makes it
  // the containing block for any position:fixed descendant — the
  // overlay was shifted 64px down (below the sidebar brand bar), its
  // right edge was clipped at the 83vw sidebar width, the close button
  // was cut off, and the register form ran past the bottom of the
  // screen with nowhere to scroll. Portaled, the overlay is measured
  // against the viewport itself at every width.
  return createPortal(
    <div
      className="auth-modal-overlay"
      onClick={onClose}
      role="dialog"
      aria-modal="true"
      aria-label={view === 'login' ? 'Sign in' : 'Create account'}
    >
      <div
        className="auth-modal-container"
        onClick={(event) => event.stopPropagation()}
      >
        <button
          className="auth-modal-close"
          onClick={onClose}
          ref={closeButtonRef}
          aria-label="Close sign-in dialog"
        >
          ×
        </button>

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

      {/* EMBEDDED STYLES */}
      <style>{`
        /* Hallmark · scope: component · genre: modern-minimal · theme: user palette (white/sea-green/black) · job: sign in / create account · states: 8 · contrast: pass · v2.0.0 */
        /* Hallmark · pre-emit critique: P4 H4 E4 S4 R4 V4 · weakest: H, the form must hold every field at 375px; capped height with internal scroll keeps the tall register form reachable */
        /* Hallmark · dismissed 29 · the dimmed backdrop blur is the modal glass Muhammad's taste profile asks for on the floating layer */
        /* Hallmark · answered F24 · the sea-green hairline is the modal edge on the near-black backdrop, and the shadow lifts the panel off the dimmed page */
        :root {
          --auth-modal-backdrop: rgba(0, 0, 0, 0.85);
          --auth-modal-bg: #1E2A38;
          --auth-modal-shadow: rgba(0, 0, 0, 0.5);
          --auth-modal-edge: #2ECC71;
          --auth-modal-muted: #94a3b8;
          --auth-modal-text: #ffffff;
        }
        .auth-modal-overlay {
          position: fixed;
          inset: 0;
          background-color: var(--auth-modal-backdrop);
          display: flex;
          justify-content: center;
          align-items: center;
          z-index: 99999;
          backdrop-filter: blur(5px);
          padding: 16px;
          box-sizing: border-box;
          overflow-y: auto;
        }
        .auth-modal-container {
          background-color: var(--auth-modal-bg);
          padding: 2rem;
          border-radius: 16px;
          width: 100%;
          max-width: 420px;
          max-height: calc(100dvh - 32px);
          overflow-y: auto;
          box-sizing: border-box;
          margin: auto;
          position: relative;
          box-shadow: 0 20px 50px var(--auth-modal-shadow);
          border: 1px solid var(--auth-modal-edge);
          color: var(--auth-modal-text);
          animation: fadeIn 0.3s ease-out;
        }
        .auth-modal-close {
          position: absolute;
          top: 12px;
          right: 16px;
          background: none;
          border: none;
          font-size: 28px;
          line-height: 1;
          min-width: 44px;
          min-height: 44px;
          color: var(--auth-modal-muted);
          cursor: pointer;
          transition: color 0.2s;
        }
        .auth-modal-close:hover {
          color: var(--auth-modal-text);
        }
        .auth-modal-close:focus-visible {
          outline: 2px solid var(--auth-modal-edge);
          outline-offset: 2px;
          color: var(--auth-modal-text);
        }
        @media (max-width: 480px) {
          .auth-modal-container {
            padding: 1.5rem;
          }
        }
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(-20px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @media (prefers-reduced-motion: reduce) {
          .auth-modal-container {
            animation: none;
          }
        }
      `}</style>
    </div>,
    document.body
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
