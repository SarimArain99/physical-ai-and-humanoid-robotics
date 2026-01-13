/**
 * Backend API Configuration
 *
 * IMPORTANT: The URL is split into parts to prevent webpack from removing
 * the protocol during minification. Docusaurus/webpack strips 'https://' from
 * string literals, so we construct it dynamically.
 */

// Split protocol to prevent webpack removal
const PROTOCOL = 'https' + ':' + '//';
const DOMAIN = 'sarimarain-ai-native-book.hf.space';

export const API_BASE_URL = PROTOCOL + DOMAIN;

// Helper function to build API URLs
export const buildApiUrl = (path) => {
  return `${API_BASE_URL}/api/${path}`.replace(/\/+/g, '/');
};

// Auth endpoints
export const authUrls = {
  getSession: () => buildApiUrl('auth/get-session'),
  signInEmail: () => buildApiUrl('auth/sign-in/email'),
  signUp: () => buildApiUrl('auth/sign-up'),
  profileUpdate: () => buildApiUrl('auth/profile/update'),
};

// Chat endpoints
export const chatUrls = {
  createSession: () => buildApiUrl('chat/sessions'),
  getSession: (sessionId) => buildApiUrl(`chat/sessions/${sessionId}`),
  listSessions: (limit = 1, offset = 0) =>
    buildApiUrl(`chat/sessions?limit=${limit}&offset=${offset}`),
  updateTitle: (sessionId) => buildApiUrl(`chat/sessions/${sessionId}/title`),
  deleteSession: (sessionId) => buildApiUrl(`chat/sessions/${sessionId}`),
  exportSession: (sessionId, format = 'json') =>
    buildApiUrl(`chat/sessions/${sessionId}/export?format=${format}`),
  chat: () => `${API_BASE_URL}/chat`,
};

// Content endpoints (Railway-specific, may not be available on HF)
export const contentUrls = {
  adjustContentBatch: () => `${API_BASE_URL}/adjust-content/batch`,
  adjustContent: () => `${API_BASE_URL}/adjust-content`,
  translateBatch: () => `${API_BASE_URL}/translate/batch`,
  translate: () => `${API_BASE_URL}/translate`,
};
