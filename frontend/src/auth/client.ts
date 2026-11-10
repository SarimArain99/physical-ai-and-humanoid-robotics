import { createAuthClient } from "better-auth/react";
import { API_BASE_URL } from "../config/api";

/**
 * 1. Create the client using the React adapter.
 * IMPORTANT: Better Auth appends '/api/auth' automatically to the baseURL.
 * API_BASE_URL is "" in production, so the client resolves to the current
 * origin (same-origin via the Vercel /api/* rewrite — see
 * frontend/vercel.json and src/config/api.js). The session cookie is
 * therefore first-party; calling the backend domain directly used to
 * make it third-party and Android Chrome blocked it, so mobile sign-in
 * never stuck. Local development uses http://localhost:8000.
 */
export const authClient = createAuthClient({
  baseURL: API_BASE_URL, 
});

// 2. Export the hooks and functions directly from the client instance
export const { useSession, signIn, signOut, signUp } = authClient;