import React, { createContext, useContext, useState, useEffect } from "react";
// We don't use useSession anymore, we manage it manually
import { authClient } from "../../auth/client";

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // 1. ON LOAD: Check if we have a token saved
  useEffect(() => {
    checkUser();
  }, []);

  const checkUser = async () => {
    const token = localStorage.getItem("auth_token");

    if (!token) {
      setLoading(false);
      return;
    }

    try {
      // Manually fetch the session using the stored token
      const response = await fetch(
        "http://localhost:8000/api/auth/get-session",
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      if (response.ok) {
        const data = await response.json();
        if (data.user) {
          console.log("Session Restored:", data.user);
          setUser(data.user);
        } else {
          logout(); // Token invalid
        }
      } else {
        logout(); // Token expired/invalid
      }
    } catch (error) {
      console.error("Session check failed", error);
      logout();
    } finally {
      setLoading(false);
    }
  };

  const login = async (email, password) => {
    try {
      // Manual Fetch for complete control
      const response = await fetch(
        "http://localhost:8000/api/auth/sign-in/email",
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ email, password }),
        }
      );

      const data = await response.json();

      if (!response.ok) {
        return { success: false, message: data.detail || "Login failed" };
      }

      // 🟢 KEY FIX: Save Token to LocalStorage
      if (data.session && data.session.accessToken) {
        localStorage.setItem("auth_token", data.session.accessToken);
        setUser(data.user);
        return { success: true };
      }

      return { success: false, message: "No token received from server" };
    } catch (error) {
      console.error("Login error:", error);
      return { success: false, message: error.message };
    }
  };

  const register = async (email, name, password,proficiency) => {
    try {
      const response = await fetch("http://localhost:8000/api/auth/sign-up", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ email, name, password, proficiency }),
      });

      const data = await response.json();

      if (!response.ok) {
        return {
          success: false,
          message: data.detail || "Registration failed",
        };
      }

      // Auto-login after register
      if (data.session && data.session.accessToken) {
        localStorage.setItem("auth_token", data.session.accessToken);
        setUser(data.user);
        return { success: true };
      }

      return { success: true };
    } catch (error) {
      console.error("Registration error:", error);
      return { success: false, message: error.message };
    }
  };

  const logout = async () => {
    localStorage.removeItem("auth_token");
    setUser(null);
    return { success: true };
  };

  const updateProfile = async (profileData) => {
    const token = localStorage.getItem("auth_token");
    if (!token) return { success: false, message: "Not logged in" };

    try {
      const response = await fetch(
        "http://localhost:8000/api/auth/profile/update",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${token}`,
          },
          body: JSON.stringify(profileData),
        }
      );

      if (!response.ok) {
        return { success: false, message: "Failed to update" };
      }

      return { success: true };
    } catch (error) {
      return { success: false, message: error.message };
    }
  };

  const value = {
    user,
    login,
    register,
    logout,
    updateProfile,
    isAuthenticated: !!user,
    loading,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};
