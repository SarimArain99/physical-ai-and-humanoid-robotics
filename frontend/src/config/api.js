/**
 * Backend API Configuration
 */

// Use environment variable for flexibility, fallback to localhost for development
export const API_BASE_URL = process.env.API_BASE_URL || 'http://localhost:8080';

/**
 * Helper function to build API URLs.
 * It ensures that the protocol 'https://' remains untouched while
 * cleaning up double slashes in the path.
 */
export const buildApiUrl = (path) => {
  // We build the path, then clean up any internal double slashes (//)
  // but protect the one in 'https://'
  const fullUrl = `${API_BASE_URL}/api/${path}`;
  const parts = fullUrl.split('://');
  return parts[0] + '://' + parts[1].replace(/\/+/g, '/');
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

// Content endpoints
export const contentUrls = {
  adjustContentBatch: () => `${API_BASE_URL}/adjust-content/batch`,
  adjustContent: () => `${API_BASE_URL}/adjust-content`,
  translateBatch: () => `${API_BASE_URL}/translate/batch`,
  translate: () => `${API_BASE_URL}/translate`,
};
