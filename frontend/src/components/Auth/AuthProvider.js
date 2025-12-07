import React, { createContext, useContext, useState, useEffect } from "react";
// We don't import client here to avoid issues

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  // Only throw if context is really missing in browser
  if (!context) {
    // Return dummy data during build to prevent crash
    if (typeof window === "undefined") {
      return { user: null, loading: false };
    }
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    checkUser();
  }, []);

  const checkUser = async () => {
    // ✅ Safe Access: Inside useEffect, so it only runs in Browser
    const token = localStorage.getItem("auth_token");

    if (!token) {
      setLoading(false);
      return;
    }

    try {
      // ✅ Use Railway URL
      const response = await fetch(
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth/get-session",
        {
          headers: {
            Authorization: `Bearer ${token}`,
          },
        }
      );

      if (response.ok) {
        const data = await response.json();
        if (data.user) {
          setUser(data.user);
        } else {
          localStorage.removeItem("auth_token");
        }
      } else {
        localStorage.removeItem("auth_token");
      }
    } catch (error) {
      console.error("Session check failed", error);
      localStorage.removeItem("auth_token");
    } finally {
      setLoading(false);
    }
  };

  const login = async (email, password) => {
    try {
      const response = await fetch(
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth/sign-in/email",
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

      if (data.session && data.session.accessToken) {
        localStorage.setItem("auth_token", data.session.accessToken);
        setUser(data.user);
        return { success: true };
      }
      return { success: false, message: "No token received" };
    } catch (error) {
      return { success: false, message: error.message };
    }
  };

  const register = async (email, name, password, proficiency) => {
    try {
      const response = await fetch(
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth/sign-up",
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ email, name, password, proficiency }),
        }
      );

      const data = await response.json();
      if (!response.ok) {
        return {
          success: false,
          message: data.detail || "Registration failed",
        };
      }

      if (data.session && data.session.accessToken) {
        localStorage.setItem("auth_token", data.session.accessToken);
        setUser(data.user);
        return { success: true };
      }
      return { success: true };
    } catch (error) {
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
        "https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth/profile/update",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${token}`,
          },
          body: JSON.stringify(profileData),
        }
      );
      if (!response.ok) return { success: false, message: "Failed to update" };
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
