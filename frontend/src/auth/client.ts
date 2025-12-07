import { createAuthClient } from "better-auth/react";

// 1. Create the client using the React adapter directly
export const authClient = createAuthClient({
  // 🔴 FIX: Must match the prefix="/api/auth" from your main.py
  baseURL: "https://physical-ai-and-humanoid-robotics-production.up.railway.app/api/auth",
});

// 2. Export the hooks and functions directly from the client instance
export const { useSession, signIn, signOut, signUp } = authClient;
