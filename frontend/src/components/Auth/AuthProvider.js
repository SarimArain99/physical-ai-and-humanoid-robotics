import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from "react";
import { authClient } from "../../auth/client";

// The backend (better-auth) stores the whole profile on the user row:
// softwareBg, hardwareBg, proficiency and learningGoals (see
// chatbot-api-node/auth.js). The server is the source of truth for all
// four. The localStorage keys below are only a legacy cache: they are
// read when an older account has no server value yet, and written after
// a successful sign-up / profile save so a slow session fetch never
// shows a blank profile. Nothing profile-related is localStorage-only.
const STORED_USER_KEY = "physical-ai-user";
const STORED_PROFICIENCY_KEY = "physical-ai-proficiency";
const STORED_GOALS_KEY = "physical-ai-learning-goals";

const VALID_PROFICIENCIES = ["beginner", "intermediate", "pro"];

// The signup form, the profile page and the Level pill all use the same
// three levels. "advanced" is an older synonym for "pro".
function normalizeProficiency(value) {
  if (typeof value !== "string") return null;
  const level = value.trim().toLowerCase();
  if (level === "advanced") return "pro";
  return VALID_PROFICIENCIES.includes(level) ? level : null;
}

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
// rest of the app sees one consistent user object. Proficiency and
// learning goals come from the server first (they are stored on the user
// row since the signup background questions were added); the localStorage
// cache is only a fallback for accounts created before that.
function buildUser(serverUser) {
  if (!serverUser) return null;
  return {
    ...serverUser,
    technical_background: serverUser.softwareBg || "",
    hardware_access: serverUser.hardwareBg || "",
    learning_goals:
      serverUser.learningGoals || readStoredValue(STORED_GOALS_KEY),
    proficiency:
      normalizeProficiency(serverUser.proficiency) ||
      normalizeProficiency(readStoredValue(STORED_PROFICIENCY_KEY)) ||
      "beginner",
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
  // Latest user, so updateProfile can merge a partial update over the
  // current profile instead of blanking fields it was not given.
  const userRef = useRef(null);
  useEffect(() => {
    userRef.current = user;
  }, [user]);

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

  // details: { proficiency, softwareBg, hardwareBg } — the background
  // questions asked at signup. They are sent to the server with the
  // sign-up request (better-auth stores them on the user row) so the
  // answers are never localStorage-only. The legacy /register page calls
  // register with three arguments; details then defaults to {}.
  const register = useCallback(async (email, name, password, details = {}) => {
    try {
      const proficiency = normalizeProficiency(details.proficiency) || "beginner";
      const softwareBg = (details.softwareBg || "").trim();
      const hardwareBg = (details.hardwareBg || "").trim();
      const { data, error } = await authClient.signUp.email({
        email,
        password,
        name,
        proficiency,
        softwareBg,
        hardwareBg,
      });
      if (error) {
        return { success: false, message: error.message || "Could not create the account. Try a different email." };
      }
      const serverUser = data && data.user;
      if (!serverUser) {
        return { success: false, message: "Account creation failed. Please try again." };
      }
      // Legacy cache only — the server row is the source of truth.
      try {
        localStorage.setItem(STORED_PROFICIENCY_KEY, proficiency);
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

  // Partial updates are merged over the current profile, so a caller
  // that only changes one field (e.g. the Level pill saving a new
  // proficiency) cannot blank the others. All four fields are stored on
  // the server user row.
  const updateProfile = useCallback(async (profileData) => {
    try {
      const current = userRef.current || {};
      const mergedData = {
        technical_background:
          profileData.technical_background ?? current.technical_background ?? "",
        hardware_access:
          profileData.hardware_access ?? current.hardware_access ?? "",
        learning_goals:
          profileData.learning_goals ?? current.learning_goals ?? "",
        proficiency:
          normalizeProficiency(profileData.proficiency) ||
          normalizeProficiency(current.proficiency) ||
          "beginner",
      };
      const { error } = await authClient.updateUser({
        softwareBg: mergedData.technical_background,
        hardwareBg: mergedData.hardware_access,
        learningGoals: mergedData.learning_goals,
        proficiency: mergedData.proficiency,
      });
      if (error) {
        return { success: false, message: error.message || "Failed to update" };
      }
      try {
        localStorage.setItem(STORED_GOALS_KEY, mergedData.learning_goals);
        localStorage.setItem(STORED_PROFICIENCY_KEY, mergedData.proficiency);
      } catch {
        // Ignore storage errors.
      }
      setUser((prev) =>
        prev
          ? {
              ...prev,
              technical_background: mergedData.technical_background,
              hardware_access: mergedData.hardware_access,
              learning_goals: mergedData.learning_goals,
              proficiency: mergedData.proficiency,
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
