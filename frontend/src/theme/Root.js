import React from "react";
import { AuthProvider } from "../components/Auth/AuthProvider"; // Adjust path if needed

// Default implementation, that you can customize
export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
