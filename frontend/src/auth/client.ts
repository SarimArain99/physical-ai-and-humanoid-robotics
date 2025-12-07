import { createAuthClient } from "better-auth/react";

// 1. Create the client using the React adapter directly
export const authClient = createAuthClient({
  // 🔴 FIX: Must match the prefix="/api/auth" from your main.py
  baseURL: "http://localhost:8000/api/auth",
});

// 2. Export the hooks and functions directly from the client instance
export const { useSession, signIn, signOut, signUp } = authClient;
