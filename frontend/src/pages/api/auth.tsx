// /src/pages/api/auth.tsx
// Docusaurus doesn't support API routes in the same way as Next.js
// This is a placeholder file - Better Auth API endpoints should be served from the backend server
// The frontend will make requests to the backend server for authentication

// This file exists to satisfy any imports but won't be used in Docusaurus
export default function handler(req, res) {
  res.status(404).json({ error: "Better Auth endpoints are served from the backend server" });
}