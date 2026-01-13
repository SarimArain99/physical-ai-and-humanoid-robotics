import { createAuthClient } from "better-auth/react";
import { API_BASE_URL } from "../config/api";

// 1. Create the client using the React adapter directly
export const authClient = createAuthClient({
  baseURL: `${API_BASE_URL}/api/auth`,
});

// 2. Export the hooks and functions directly from the client instance
export const { useSession, signIn, signOut, signUp } = authClient;
