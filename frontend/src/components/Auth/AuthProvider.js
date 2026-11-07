import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from "react";
import { authClient } from "../../auth/client";

// The backend (better-auth) only stores two profile fields: softwareBg and
// hardwareBg. The profile page also asks for learning goals and a reading
// level, which have no backend field, so those live in localStorage.
const STORED_USER_KEY = "physical-ai-user";
const STORED_PROFICIENCY_KEY = "physical-ai-proficiency";
const STORED_GOALS_KEY = "physical-ai-learning-goals";

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    // Return dummy data during build to prevent crash
    if (typeof window === "undefined") {
      return { user: null, loading: false };
    }
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
};

function readStoredUser() {
  if (typeof window === "undefined") return null;
  try {
    const raw = localStorage.getItem(STORED_USER_KEY);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

function readStoredValue(key) {
  if (typeof window === "undefined") return "";
  try {
    return localStorage.getItem(key) || "";
  } catch {
    return "";
  }
}

function storeUser(user) {
  try {
    if (user) {
      localStorage.setItem(STORED_USER_KEY, JSON.stringify(user));
    } else {
      localStorage.removeItem(STORED_USER_KEY);
    }
  } catch {
    // Storage unavailable — the session still works for this visit.
  }
}

// Merge the server user with the locally stored profile fields so the
// rest of the app sees one consistent user object.
function buildUser(serverUser) {
  if (!serverUser) return null;
  return {
    ...serverUser,
    technical_background: serverUser.softwareBg || "",
    hardware_access: serverUser.hardwareBg || "",
    learning_goals: readStoredValue(STORED_GOALS_KEY),
    proficiency: readStoredValue(STORED_PROFICIENCY_KEY) || "beginner",
  };
}

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const { data: sessionData, isPending: sessionPending } = authClient.useSession();
  // True after a successful sign-in/sign-up in this visit, so a missing
  // server session does not immediately undo a fresh login (e.g. when the
  // browser blocks third-party cookies). A fresh visit re-checks the server.
  const localSessionRef = useRef(false);

  // Restore the stored user on mount (client only, avoids SSR mismatch).
  useEffect(() => {
    const stored = readStoredUser();
    if (stored) setUser(stored);
  }, []);

  // Reconcile with the server session once it has been checked.
  useEffect(() => {
    if (sessionPending) return;
    if (sessionData && sessionData.user) {
      const merged = buildUser(sessionData.user);
      setUser(merged);
      storeUser(merged);
    } else if (!localSessionRef.current) {
      // The server says there is no session and nobody signed in during
      // this visit — the stored user (if any) is stale.
      storeUser(null);
      setUser(null);
    }
    setLoading(false);
  }, [sessionData, sessionPending]);

  const login = useCallback(async (email, password) => {
    try {
      const { data, error } = await authClient.signIn.email({ email, password });
      if (error) {
        return { success: false, message: error.message || "Sign-in failed. Check your email and password." };
      }
      const serverUser = data && data.user;
      if (!serverUser) {
        return { success: false, message: "Sign-in failed. Please try again." };
      }
      const merged = buildUser(serverUser);
      localSessionRef.current = true;
      setUser(merged);
      storeUser(merged);
      return { success: true };
    } catch (err) {
      return { success: false, message: "Could not reach the sign-in service. Check your connection and try again." };
    }
  }, []);

  const register = useCallback(async (email, name, password, proficiency) => {
    try {
      const { data, error } = await authClient.signUp.email({
        email,
        password,
        name,
      });
      if (error) {
        return { success: false, message: error.message || "Could not create the account. Try a different email." };
      }
      const serverUser = data && data.user;
      if (!serverUser) {
        return { success: false, message: "Account creation failed. Please try again." };
      }
      try {
        localStorage.setItem(STORED_PROFICIENCY_KEY, proficiency || "beginner");
      } catch {
        // Ignore storage errors.
      }
      const merged = buildUser(serverUser);
      localSessionRef.current = true;
      setUser(merged);
      storeUser(merged);
      return { success: true };
    } catch (err) {
      return { success: false, message: "Could not reach the sign-up service. Check your connection and try again." };
    }
  }, []);

  const logout = useCallback(async () => {
    try {
      await authClient.signOut();
    } catch {
      // Even if the server call fails, sign out locally.
    }
    localSessionRef.current = false;
    storeUser(null);
    setUser(null);
  }, []);

  const updateProfile = useCallback(async (profileData) => {
    try {
      const { error } = await authClient.updateUser({
        softwareBg: profileData.technical_background || "",
        hardwareBg: profileData.hardware_access || "",
      });
      if (error) {
        return { success: false, message: error.message || "Failed to update" };
      }
      try {
        localStorage.setItem(STORED_GOALS_KEY, profileData.learning_goals || "");
        if (profileData.proficiency) {
          localStorage.setItem(STORED_PROFICIENCY_KEY, profileData.proficiency);
        }
      } catch {
        // Ignore storage errors.
      }
      setUser((prev) =>
        prev
          ? {
              ...prev,
              technical_background: profileData.technical_background || "",
              hardware_access: profileData.hardware_access || "",
              learning_goals: profileData.learning_goals || "",
            }
          : prev
      );
      return { success: true };
    } catch (err) {
      return { success: false, message: "Could not reach the profile service. Check your connection and try again." };
    }
  }, []);

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
