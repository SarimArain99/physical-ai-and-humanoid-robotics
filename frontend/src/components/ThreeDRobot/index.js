import React, { useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import './styles.css';

/**
 * ThreeDRobot Component
 * Feature: 002-ui-improvements
 * Task: T054, T057, T058, T059, T061
 *
 * Interactive 3D humanoid robot model using Three.js
 * - WebGL detection with SVG fallback
 * - Auto-rotate animation
 * - Mouse/touch drag controls
 * - Performance-based quality adjustment
 * - FPS monitoring (T061)
 */

// Simple robot geometry data
const ROBOT_CONFIG = {
  head: { radius: 0.5, segments: 16 },
  body: { width: 0.8, height: 1.2, depth: 0.5 },
  arms: { width: 0.2, length: 0.8 },
  legs: { width: 0.25, length: 1.0 },
  colors: {
    primary: 0x2ECC71,
    secondary: 0x1E2A38,
    accent: 0x4ECDC4,
    joints: 0x64748B
  }
};

function hasWebGL() {
  if (typeof window === 'undefined') return false;
  try {
    const canvas = document.createElement('canvas');
    return !!(canvas.getContext('webgl') || canvas.getContext('experimental-webgl'));
  } catch {
    return false;
  }
}

function getPerformanceLevel() {
  if (typeof window === 'undefined') return 'low';
  if (window.matchMedia?.('(prefers-reduced-motion: reduce)').matches) return 'low';
  const cores = navigator.hardwareConcurrency || 2;
  const memory = navigator.deviceMemory || 4;
  if (cores >= 8 && memory >= 8) return 'high';
  if (cores >= 4 && memory >= 4) return 'medium';
  return 'low';
}

export default function ThreeDRobot({
  autoRotate = true,
  rotateSpeed = 0.005,
  enableDrag = true,
  width = 300,
  height = 400,
  className = '',
  showPerfStats = false // T061: Show performance stats in development
}) {
  const containerRef = useRef(null);
  const cleanupRef = useRef(null);
  const [webglSupported, setWebglSupported] = useState(null);
  const [performanceLevel, setPerformanceLevel] = useState('medium');
  const [fps, setFps] = useState(null); // T061: FPS tracking

  // Single detection effect on mount
  useEffect(() => {
    const supported = hasWebGL();
    setWebglSupported(supported);
    setPerformanceLevel(getPerformanceLevel());
    if (!supported) console.info('WebGL not supported, showing fallback');
  }, []);

  // Main initialization effect - runs only after WebGL is confirmed
  useEffect(() => {
    if (webglSupported !== true) return;
    if (!containerRef.current) return;

    // Prevent double initialization
    if (cleanupRef.current) return;

    const container = containerRef.current;
    const isLowPerf = performanceLevel === 'low';

    // Setup Three.js
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
    camera.position.z = 5;
    camera.position.y = 0.5;

    const renderer = new THREE.WebGLRenderer({
      antialias: !isLowPerf,
      alpha: true,
      powerPreference: 'high-performance'
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, isLowPerf ? 1 : 2));
    container.appendChild(renderer.domElement);

    // Lighting
    scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(5, 5, 5);
    scene.add(dirLight);
    const backLight = new THREE.DirectionalLight(0x4ECDC4, 0.3);
    backLight.position.set(-5, 3, -5);
    scene.add(backLight);

    // Materials
    const primaryMat = new THREE.MeshPhongMaterial({
      color: ROBOT_CONFIG.colors.primary,
      shininess: 100,
      flatShading: isLowPerf
    });
    const secondaryMat = new THREE.MeshPhongMaterial({
      color: ROBOT_CONFIG.colors.secondary,
      shininess: 50,
      flatShading: isLowPerf
    });
    const accentMat = new THREE.MeshPhongMaterial({
      color: ROBOT_CONFIG.colors.accent,
      shininess: 80,
      emissive: ROBOT_CONFIG.colors.accent,
      emissiveIntensity: 0.2
    });

    // Create robot
    const robot = new THREE.Group();

    // Head
    const head = new THREE.Mesh(
      new THREE.SphereGeometry(ROBOT_CONFIG.head.radius, isLowPerf ? 8 : 16, isLowPerf ? 8 : 12),
      primaryMat
    );
    head.position.y = 1.8;
    robot.add(head);

    // Visor
    const visor = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.15, 0.3), accentMat);
    visor.position.set(0, 1.8, 0.35);
    robot.add(visor);

    // Eyes
    const eyeGeom = new THREE.SphereGeometry(0.08, 8, 8);
    const eyeMat = new THREE.MeshBasicMaterial({ color: 0x00FF00 });
    const leftEye = new THREE.Mesh(eyeGeom, eyeMat);
    leftEye.position.set(-0.15, 1.85, 0.42);
    robot.add(leftEye);
    const rightEye = new THREE.Mesh(eyeGeom, eyeMat);
    rightEye.position.set(0.15, 1.85, 0.42);
    robot.add(rightEye);

    // Body
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(ROBOT_CONFIG.body.width, ROBOT_CONFIG.body.height, ROBOT_CONFIG.body.depth),
      secondaryMat
    );
    body.position.y = 0.8;
    robot.add(body);

    // Chest
    const chest = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.4, 0.1), accentMat);
    chest.position.set(0, 1.0, 0.28);
    robot.add(chest);

    // Arms
    const armGeom = new THREE.BoxGeometry(ROBOT_CONFIG.arms.width, ROBOT_CONFIG.arms.length, ROBOT_CONFIG.arms.width);
    const leftArm = new THREE.Mesh(armGeom, secondaryMat);
    leftArm.position.set(-0.6, 0.7, 0);
    leftArm.rotation.z = 0.1;
    robot.add(leftArm);
    const rightArm = new THREE.Mesh(armGeom, secondaryMat);
    rightArm.position.set(0.6, 0.7, 0);
    rightArm.rotation.z = -0.1;
    robot.add(rightArm);

    // Hands
    const handGeom = new THREE.BoxGeometry(0.15, 0.2, 0.15);
    const leftHand = new THREE.Mesh(handGeom, primaryMat);
    leftHand.position.set(-0.6, 0.15, 0);
    robot.add(leftHand);
    const rightHand = new THREE.Mesh(handGeom, primaryMat);
    rightHand.position.set(0.6, 0.15, 0);
    robot.add(rightHand);

    // Legs
    const legGeom = new THREE.BoxGeometry(ROBOT_CONFIG.legs.width, ROBOT_CONFIG.legs.length, ROBOT_CONFIG.legs.width);
    const leftLeg = new THREE.Mesh(legGeom, secondaryMat);
    leftLeg.position.set(-0.25, -0.4, 0);
    robot.add(leftLeg);
    const rightLeg = new THREE.Mesh(legGeom, secondaryMat);
    rightLeg.position.set(0.25, -0.4, 0);
    robot.add(rightLeg);

    // Feet
    const footGeom = new THREE.BoxGeometry(0.3, 0.15, 0.4);
    const leftFoot = new THREE.Mesh(footGeom, primaryMat);
    leftFoot.position.set(-0.25, -1.0, 0.05);
    robot.add(leftFoot);
    const rightFoot = new THREE.Mesh(footGeom, primaryMat);
    rightFoot.position.set(0.25, -1.0, 0.05);
    robot.add(rightFoot);

    robot.rotation.y = 0.3;
    scene.add(robot);

    // Animation state
    let isDragging = false;
    let previousMouse = { x: 0, y: 0 };
    let animationId;

    // T061: FPS tracking
    let frameCount = 0;
    let lastFpsUpdate = performance.now();
    const fpsUpdateInterval = 1000; // Update FPS every second

    // Animation loop
    const animate = () => {
      animationId = requestAnimationFrame(animate);
      if (!isDragging && autoRotate) {
        robot.rotation.y += rotateSpeed;
      }
      renderer.render(scene, camera);

      // T061: Calculate FPS
      frameCount++;
      const now = performance.now();
      if (now - lastFpsUpdate >= fpsUpdateInterval) {
        const currentFps = Math.round((frameCount * 1000) / (now - lastFpsUpdate));
        if (showPerfStats) {
          setFps(currentFps);
        }
        // Log low FPS warnings in development
        if (process.env.NODE_ENV === 'development' && currentFps < 30) {
          console.warn(`[ThreeDRobot] Low FPS detected: ${currentFps} FPS`);
        }
        frameCount = 0;
        lastFpsUpdate = now;
      }
    };
    animate();

    // Event handlers
    const handleMouseDown = (e) => {
      if (!enableDrag) return;
      isDragging = true;
      previousMouse = { x: e.clientX, y: e.clientY };
    };

    const handleMouseMove = (e) => {
      if (!isDragging) return;
      const dx = e.clientX - previousMouse.x;
      const dy = e.clientY - previousMouse.y;
      robot.rotation.y += dx * 0.01;
      robot.rotation.x += dy * 0.01;
      robot.rotation.x = Math.max(-Math.PI / 4, Math.min(Math.PI / 4, robot.rotation.x));
      previousMouse = { x: e.clientX, y: e.clientY };
    };

    const handleMouseUp = () => {
      isDragging = false;
    };

    const handleTouchStart = (e) => {
      if (!enableDrag || e.touches.length !== 1) return;
      isDragging = true;
      previousMouse = { x: e.touches[0].clientX, y: e.touches[0].clientY };
    };

    const handleTouchMove = (e) => {
      if (!isDragging) return;
      e.preventDefault();
      const dx = e.touches[0].clientX - previousMouse.x;
      const dy = e.touches[0].clientY - previousMouse.y;
      robot.rotation.y += dx * 0.01;
      robot.rotation.x += dy * 0.01;
      robot.rotation.x = Math.max(-Math.PI / 4, Math.min(Math.PI / 4, robot.rotation.x));
      previousMouse = { x: e.touches[0].clientX, y: e.touches[0].clientY };
    };

    const handleTouchEnd = () => {
      isDragging = false;
    };

    // Add listeners
    container.addEventListener('mousedown', handleMouseDown);
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
    container.addEventListener('touchstart', handleTouchStart, { passive: false });
    window.addEventListener('touchmove', handleTouchMove, { passive: false });
    window.addEventListener('touchend', handleTouchEnd);

    // Store cleanup function
    cleanupRef.current = () => {
      cancelAnimationFrame(animationId);
      if (renderer.domElement?.parentNode === container) {
        container.removeChild(renderer.domElement);
      }
      renderer.dispose();
      container.removeEventListener('mousedown', handleMouseDown);
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
      container.removeEventListener('touchstart', handleTouchStart);
      window.removeEventListener('touchmove', handleTouchMove);
      window.removeEventListener('touchend', handleTouchEnd);
      cleanupRef.current = null;
    };

    // Cleanup on unmount
    return () => {
      cleanupRef.current?.();
    };
  }, [webglSupported, performanceLevel, autoRotate, rotateSpeed, width, height, enableDrag, showPerfStats]);

  // Show fallback while loading or if WebGL not supported
  if (webglSupported === null) {
    return (
      <div className={`three-d-robot-container ${className}`} style={{ width, height }} role="img" aria-label="3D Robot illustration">
        <div className="three-d-robot-loading">Loading...</div>
      </div>
    );
  }

  if (webglSupported === false) {
    return (
      <div className={`three-d-robot-container ${className}`} style={{ width, height }} role="img" aria-label="3D Robot illustration">
        <FallbackSVG width={width} height={height} />
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className={`three-d-robot-container three-d-robot-interactive ${className}`}
      style={{ width, height }}
      role="img"
      aria-label="Interactive 3D Robot model. Drag to rotate."
      tabIndex={enableDrag ? 0 : undefined}
    >
      {showPerfStats && fps !== null && (
        <div className="three-d-robot-fps" style={{
          position: 'absolute',
          top: '4px',
          left: '4px',
          background: 'rgba(0,0,0,0.7)',
          color: fps >= 30 ? '#2ECC71' : '#FF6B35',
          padding: '2px 6px',
          borderRadius: '4px',
          fontSize: '10px',
          fontFamily: 'monospace',
          pointerEvents: 'none'
        }}>
          {fps} FPS
        </div>
      )}
    </div>
  );
}

/**
 * Fallback SVG for no WebGL support
 */
function FallbackSVG({ width = 300, height = 400 }) {
  return (
    <svg
      width={width}
      height={height}
      viewBox="0 0 300 400"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
      aria-hidden="true"
      className="three-d-robot-fallback-svg"
    >
      {/* Robot body */}
      <rect x="100" y="120" width="100" height="120" rx="10" fill="#1E2A38" />
      <rect x="115" y="135" width="70" height="40" rx="5" fill="#4ECDC4" />

      {/* Head */}
      <circle cx="150" cy="80" r="40" fill="#2ECC71" />
      <rect x="110" y="70" width="80" height="20" rx="5" fill="#4ECDC4" />
      <circle cx="130" cy="80" r="8" fill="#00FF00" />
      <circle cx="170" cy="80" r="8" fill="#00FF00" />

      {/* Arms */}
      <rect x="60" y="125" width="30" height="80" rx="8" fill="#1E2A38" />
      <rect x="210" y="125" width="30" height="80" rx="8" fill="#1E2A38" />
      <rect x="60" y="200" width="30" height="30" rx="5" fill="#2ECC71" />
      <rect x="210" y="200" width="30" height="30" rx="5" fill="#2ECC71" />

      {/* Legs */}
      <rect x="110" y="235" width="30" height="100" rx="8" fill="#1E2A38" />
      <rect x="160" y="235" width="30" height="100" rx="8" fill="#1E2A38" />
      <rect x="100" y="330" width="50" height="25" rx="5" fill="#2ECC71" />
      <rect x="150" y="330" width="50" height="25" rx="5" fill="#2ECC71" />

      {/* Glow effect */}
      <circle cx="150" cy="200" r="80" fill="url(#glow)" opacity="0.3" />

      <defs>
        <radialGradient id="glow" cx="0" cy="0" r="1">
          <stop offset="0%" stopColor="#4ECDC4" />
          <stop offset="100%" stopColor="#4ECDC4" stopOpacity="0" />
        </radialGradient>
      </defs>
    </svg>
  );
}

export { FallbackSVG };
