import { betterAuth } from "better-auth";
import { admin } from "better-auth/plugins";
import { dash } from "@better-auth/infra";
import pg from "pg";
const { Pool } = pg;

import nodemailer from "nodemailer";
const pool = new Pool({
  connectionString: process.env.DATABASE_URL
});

// Signup background answers (hackathon spec point 5) live on the user row.
// better-auth stores additionalFields in the user table but does not create
// their columns itself, so make sure they exist. Idempotent: safe to run
// on every boot and on deploys that predate the new columns. If the
// database is unreachable this only logs — the server still starts and
// /api/health still answers; sign-up itself needs the database.
pool.query(`
  ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "softwareBg" TEXT;
  ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "hardwareBg" TEXT;
  ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "proficiency" TEXT;
  ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "learningGoals" TEXT;
`).then(() => console.log("Better Auth: profile columns ready."))
  .catch((err) => console.error("Better Auth: profile column check failed:", err.message));

const allowedOrigins = process.env.CORS_ORIGINS 
  ? process.env.CORS_ORIGINS.split(",").map(o => o.trim().replace(/\/$/, "")) 
  : [];

// Finding #1/#2: better-auth's CSRF defence is an Origin check against
// trustedOrigins. The production frontend origin is a default here (not
// only from env) so the check is correct even if CORS_ORIGINS is unset.
// The frontend is same-origin in production (Vercel /api/* rewrite),
// so this is the origin the browser actually sends. Never add a
// `.vercel.app` suffix here — see api/index.js.
const PRODUCTION_FRONTEND_ORIGIN = "https://physical-ai-and-humanoid-robotics-sarim.vercel.app";
const trustedOrigins = [
  ...new Set([
    "http://localhost:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
    PRODUCTION_FRONTEND_ORIGIN,
    ...allowedOrigins,
  ]),
];
console.log("Better Auth: Configured Trusted Origins ->", trustedOrigins);

export const auth = betterAuth({
  database: pool,
  secret: process.env.BETTER_AUTH_SECRET,
  emailAndPassword: {
    enabled: true,
    sendResetPassword: async ({ user, url }) => {
      // Finding #5: the old code logged the full reset URL (live token)
      // into the Vercel logs. Never log the URL or the token — the
      // email alone identifies the request. (The url is still used
      // below, inside the email itself, where it belongs.)
      console.log(`[PASSWORD RESET REQUEST] for ${user.email}`);

      // Finding #5 (continued): better-auth runs this callback in the
      // background, so a failure here can never change the HTTP
      // response — the user would be told to "check your inbox" for
      // mail that will never arrive. The honest error for a missing
      // transport is returned by the Express guard in api/index.js
      // (resetMailGuard), which runs before better-auth. This throw is
      // a safety net for any path that reaches here without a
      // transport, and for a failed send: if better-auth awaits the
      // callback, the error surfaces instead of a false promise.
      if (!process.env.SMTP_HOST) {
        console.error(`Password reset requested for ${user.email} but SMTP_HOST is not configured — no reset email can be sent.`);
        throw new Error("Password reset email is not available right now. Please contact support.");
      }

      try {
          const transporter = nodemailer.createTransport({
            host: process.env.SMTP_HOST,
            port: parseInt(process.env.SMTP_PORT || "587"),
            secure: process.env.SMTP_SECURE === "true",
            auth: {
              user: process.env.SMTP_USER,
              pass: process.env.SMTP_PASS
            }
          });
          
          const mailOptions = {
            from: process.env.SMTP_FROM || `"Physical AI Textbook" <noreply@yourdomain.com>`,
            to: user.email,
            subject: "Reset Your Password - Physical AI Textbook",
            html: `
              <h2>Password Reset Request</h2>
              <p>Hello ${user.name || "Student"},</p>
              <p>We received a request to reset the password for your Physical AI & Humanoid Robotics Textbook account.</p>
              <p>Click the link below to set a new password. This link is valid for 1 hour:</p>
              <a href="${url}" style="display:inline-block;background:#3b82f6;color:#fff;padding:10px 20px;text-decoration:none;border-radius:6px;font-weight:bold;">Reset Password</a>
              <br/><br/>
              <p>If you didn't request this, you can safely ignore this email.</p>
            `
          };
          
          await transporter.sendMail(mailOptions);
          console.log(`Reset email sent successfully to ${user.email}`);
      } catch (mailError) {
        // Finding #5: a failed send must not leave the user believing
        // mail is on its way. Throw so the failure surfaces (when
        // better-auth awaits this callback) instead of being swallowed.
        console.error(`Failed to send password reset email to ${user.email}:`, mailError);
        throw new Error("Password reset email could not be sent. Please try again later or contact support.");
      }
    }
  },
  trustedOrigins: trustedOrigins,
  advanced: {
    useSecureCookies: true,
    cookies: {
      session_token: {
        name: "better-auth.session_token",
        attributes: {
          // Finding #2: the cookie was sameSite "None" (cross-site),
          // which let a malicious page's requests carry it. The
          // frontend is same-origin in production (Vercel /api/*
          // rewrite) and same-site on localhost:8000 in development,
          // so "Lax" is sent on every legitimate request and NOT on
          // cross-site ones — the CSRF defence now has a second layer
          // behind the trustedOrigins check.
          sameSite: "Lax",
          secure: true,
        }
      }
    }
  },
  user: {
    additionalFields: {
      softwareBg: {
        type: "string",
        required: false,
      },
      hardwareBg: {
        type: "string",
        required: false,
      },
      // Asked at signup (with the two background questions) and shown on
      // /profile. Stored on the user row — never localStorage-only.
      proficiency: {
        type: "string",
        required: false,
      },
      learningGoals: {
        type: "string",
        required: false,
      }
    }
  },
  plugins: [
    dash(),
    admin(),
  ]
});
