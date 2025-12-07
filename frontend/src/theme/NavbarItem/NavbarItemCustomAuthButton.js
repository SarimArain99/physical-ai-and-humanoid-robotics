import React from 'react';
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const NavbarItemCustomAuthButton = () => {
  const { user, logout, isAuthenticated, loading } = useAuth();

  if (loading) {
    return <div className="navbar-auth-loading">Loading...</div>;
  }

  return (
    <>
      {isAuthenticated ? (
        <div className="navbar-auth-dropdown">
          <button
            className="navbar-auth-user-button"
            aria-label="User menu"
          >
            <span className="navbar-auth-user-name">
              {user?.name || user?.email?.split('@')[0] || 'User'}
            </span>
          </button>
          <div className="navbar-auth-dropdown-content">
            <a
              href="/profile"
              className="navbar-auth-dropdown-item-link"
            >
              <button className="navbar-auth-dropdown-item">
                Profile
              </button>
            </a>
            <button
              className="navbar-auth-dropdown-item"
              onClick={logout}
            >
              Logout
            </button>
          </div>
        </div>
      ) : (
        <a href="/login">
          <button className="navbar-auth-button">
            Sign In
          </button>
        </a>
      )}

      <style jsx>{`
        .navbar-auth-button {
          background-color: #2563eb;
          color: white;
          border: none;
          border-radius: 6px;
          padding: 0.5rem 1rem;
          cursor: pointer;
          font-weight: 500;
          transition: background-color 0.2s;
          height: 36px;
          font-size: 0.875rem;
          text-decoration: none;
        }

        .navbar-auth-button:hover {
          background-color: #1d4ed8;
        }

        .navbar-auth-user-button {
          background: none;
          border: none;
          color: #1f2937;
          cursor: pointer;
          font-weight: 500;
          display: flex;
          align-items: center;
          gap: 0.5rem;
          height: 36px;
          padding: 0 0.5rem;
          border-radius: 4px;
        }

        .navbar-auth-user-button:hover {
          color: #2563eb;
          background-color: #f3f4f6;
        }

        .navbar-auth-user-name {
          font-size: 0.875rem;
        }

        .navbar-auth-dropdown {
          position: relative;
          display: inline-block;
        }

        .navbar-auth-dropdown-content {
          display: none;
          position: absolute;
          right: 0;
          top: 100%;
          background-color: white;
          min-width: 160px;
          box-shadow: 0px 8px 16px 0px rgba(0,0,0,0.2);
          z-index: 1000;
          border-radius: 4px;
          overflow: hidden;
          margin-top: 0.25rem;
        }

        .navbar-auth-dropdown:hover .navbar-auth-dropdown-content {
          display: block;
        }

        .navbar-auth-dropdown-item-link {
          text-decoration: none;
          display: block;
          width: 100%;
        }

        .navbar-auth-dropdown-item {
          background-color: white;
          color: #1f2937;
          padding: 0.75rem 1rem;
          text-decoration: none;
          display: block;
          border: none;
          width: 100%;
          text-align: left;
          cursor: pointer;
          transition: background-color 0.2s;
          font-size: 0.875rem;
        }

        .navbar-auth-dropdown-item:hover {
          background-color: #f3f4f6;
        }

        .navbar-auth-loading {
          padding: 0.5rem 1rem;
          font-size: 0.875rem;
        }
      `}</style>
    </>
  );
};

export default NavbarItemCustomAuthButton;