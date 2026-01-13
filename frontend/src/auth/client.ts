import { createAuthClient } from "better-auth/react";
import { API_BASE_URL } from "../config/api";

/**
 * 1. Create the client using the React adapter.
 * IMPORTANT: Better Auth appends '/api/auth' automatically to the baseURL.
 * We pass the base domain (https://sarimarain-ai-native-book.hf.space) directly.
 */
export const authClient = createAuthClient({
  baseURL: API_BASE_URL, 
});

// 2. Export the hooks and functions directly from the client instance
export const { useSession, signIn, signOut, signUp } = authClient;