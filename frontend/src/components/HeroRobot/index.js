import React from 'react';

/**
 * HeroRobot — a static robot illustration for the homepage hero.
 *
 * The previous hero tried to load a live 3D model (three.js) and never
 * visibly rendered — visitors saw a solid green block with a fake
 * "Loading 3D Robot..." state. This renders instantly, with no
 * dependency, no loading state, and no network fetch.
 */
export default function HeroRobot({ width = 280, height = 350 }) {
  return (
    <div className="hero-robot" role="img" aria-label="Illustration of a humanoid robot">
      <svg
        width={width}
        height={height}
        viewBox="0 0 300 400"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
        aria-hidden="true"
        className="hero-robot-svg"
      >
        {/* Robot body */}
        <rect x="100" y="120" width="100" height="120" rx="10" fill="#1E2A38" />
        <rect x="115" y="135" width="70" height="40" rx="5" fill="#79B8FF" />

        {/* Head */}
        <circle cx="150" cy="80" r="40" fill="#58A6FF" />
        <rect x="110" y="70" width="80" height="20" rx="5" fill="#79B8FF" />
        <circle cx="130" cy="80" r="8" fill="#58A6FF" />
        <circle cx="170" cy="80" r="8" fill="#58A6FF" />

        {/* Arms */}
        <rect x="60" y="125" width="30" height="80" rx="8" fill="#1E2A38" />
        <rect x="210" y="125" width="30" height="80" rx="8" fill="#1E2A38" />
        <rect x="60" y="200" width="30" height="30" rx="5" fill="#58A6FF" />
        <rect x="210" y="200" width="30" height="30" rx="5" fill="#58A6FF" />

        {/* Legs */}
        <rect x="110" y="235" width="30" height="100" rx="8" fill="#1E2A38" />
        <rect x="160" y="235" width="30" height="100" rx="8" fill="#1E2A38" />
        <rect x="100" y="330" width="50" height="25" rx="5" fill="#58A6FF" />
        <rect x="150" y="330" width="50" height="25" rx="5" fill="#58A6FF" />

        {/* Glow effect */}
        <circle cx="150" cy="200" r="80" fill="url(#heroRobotGlow)" opacity="0.3" />

        <defs>
          <radialGradient id="heroRobotGlow" cx="0" cy="0" r="1">
            <stop offset="0%" stopColor="#79B8FF" />
            <stop offset="100%" stopColor="#79B8FF" stopOpacity="0" />
          </radialGradient>
        </defs>
      </svg>
      <style>{`
        .hero-robot { display: flex; align-items: center; justify-content: center; }
        .hero-robot-svg { animation: heroRobotFloat 5s ease-in-out infinite; }
        @keyframes heroRobotFloat {
          0%, 100% { transform: translateY(0); }
          50% { transform: translateY(-10px); }
        }
        @media (prefers-reduced-motion: reduce) {
          .hero-robot-svg { animation: none; }
        }
      `}</style>
    </div>
  );
}
