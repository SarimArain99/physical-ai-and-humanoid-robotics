import React from 'react';
import './styles.css';

/**
 * SenseThinkActLoop — the book's signature diagram, as a React component
 * for the homepage hero. Three nodes (Sense, Think, Act) joined by
 * arrows, with one pulse travelling the loop. Colours come from the
 * theme tokens, so it reads in light and dark. The same diagram ships
 * as a static SVG inside Week 1.
 */
export default function SenseThinkActLoop() {
  return (
    <svg
      className="loop-diagram"
      viewBox="0 0 640 360"
      role="img"
      aria-label="Sense Think Act loop diagram: a robot senses the world, decides what to do, acts, then senses again"
    >
      <defs>
        <marker
          id="loop-arrow"
          markerWidth="10"
          markerHeight="8"
          refX="9"
          refY="4"
          orient="auto"
        >
          <polygon points="0 0, 10 4, 0 8" className="loop-arrow-head" />
        </marker>
      </defs>

      {/* Sense -> Think */}
      <path
        className="loop-arrow"
        d="M 430 100 Q 500 160 482 224"
        markerEnd="url(#loop-arrow)"
      />
      <text className="loop-edge-label" x="545" y="145" textAnchor="middle">
        decide
      </text>

      {/* Think -> Act */}
      <path
        className="loop-arrow"
        d="M 400 262 Q 320 292 242 262"
        markerEnd="url(#loop-arrow)"
      />
      <text className="loop-edge-label" x="320" y="312" textAnchor="middle">
        command
      </text>

      {/* Act -> Sense */}
      <path
        className="loop-arrow"
        d="M 140 224 Q 148 140 218 84"
        markerEnd="url(#loop-arrow)"
      />
      <text className="loop-edge-label" x="75" y="145" textAnchor="middle">
        new reading
      </text>

      {/* Sense node */}
      <rect className="loop-node" x="220" y="38" width="200" height="64" rx="12" />
      <text className="loop-node-label" x="320" y="66" textAnchor="middle">
        Sense
      </text>
      <text className="loop-node-sub" x="320" y="88" textAnchor="middle">
        cameras, lidar, IMU
      </text>

      {/* Think node */}
      <rect className="loop-node" x="400" y="228" width="200" height="64" rx="12" />
      <text className="loop-node-label" x="500" y="256" textAnchor="middle">
        Think
      </text>
      <text className="loop-node-sub" x="500" y="278" textAnchor="middle">
        planner or learned policy
      </text>

      {/* Act node */}
      <rect className="loop-node" x="40" y="228" width="200" height="64" rx="12" />
      <text className="loop-node-label" x="140" y="256" textAnchor="middle">
        Act
      </text>
      <text className="loop-node-sub" x="140" y="278" textAnchor="middle">
        motors through ROS 2
      </text>

      <text className="loop-centre" x="320" y="200" textAnchor="middle">
        the loop repeats
      </text>

      {/* Pulse travelling the loop */}
      <circle className="loop-pulse" r="6" />
    </svg>
  );
}
