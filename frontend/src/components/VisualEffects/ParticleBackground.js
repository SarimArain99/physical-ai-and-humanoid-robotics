import React, { useRef, useEffect, useState } from 'react';
import './styles.css';

/**
 * Particle Background Component
 * Feature: 002-ui-improvements
 * Task: T080, T083, T085
 *
 * Canvas-based particle animation with:
 * - Performance-based particle count
 * - Mouse interaction
 * - Efficient rendering
 */

export function ParticleBackground({ count = 50, color = 'rgba(46, 204, 113, 0.5)' }) {
  const canvasRef = useRef(null);
  const animationRef = useRef(null);
  const particlesRef = useRef([]);
  const mouseRef = useRef({ x: null, y: null });
  const [isVisible, setIsVisible] = useState(true);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    const resizeCanvas = () => {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    };
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // Particle class
    class Particle {
      constructor() {
        this.reset();
      }

      reset() {
        this.x = Math.random() * canvas.width;
        this.y = Math.random() * canvas.height;
        this.size = Math.random() * 2 + 1;
        this.speedX = (Math.random() - 0.5) * 0.5;
        this.speedY = (Math.random() - 0.5) * 0.5;
        this.opacity = Math.random() * 0.5 + 0.2;
      }

      update() {
        this.x += this.speedX;
        this.y += this.speedY;

        // Mouse interaction
        if (mouseRef.current.x !== null) {
          const dx = mouseRef.current.x - this.x;
          const dy = mouseRef.current.y - this.y;
          const distance = Math.sqrt(dx * dx + dy * dy);
          if (distance < 150) {
            const force = (150 - distance) / 150;
            this.speedX -= (dx / distance) * force * 0.02;
            this.speedY -= (dy / distance) * force * 0.02;
          }
        }

        // Wrap around edges
        if (this.x < 0) this.x = canvas.width;
        if (this.x > canvas.width) this.x = 0;
        if (this.y < 0) this.y = canvas.height;
        if (this.y > canvas.height) this.y = 0;
      }

      draw() {
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.size, 0, Math.PI * 2);
        ctx.fillStyle = color.replace('0.5)', `${this.opacity})`);
        ctx.fill();
      }
    }

    // Initialize particles
    particlesRef.current = Array.from({ length: count }, () => new Particle());

    // Animation loop with throttling for performance
    let lastTime = 0;
    const targetFPS = 30;
    const frameInterval = 1000 / targetFPS;

    const animate = (currentTime) => {
      if (!isVisible) {
        animationRef.current = requestAnimationFrame(animate);
        return;
      }

      const deltaTime = currentTime - lastTime;

      if (deltaTime >= frameInterval) {
        lastTime = currentTime - (deltaTime % frameInterval);

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Update and draw particles
        particlesRef.current.forEach(particle => {
          particle.update();
          particle.draw();
        });

        // Draw connections between nearby particles
        particlesRef.current.forEach((p1, i) => {
          particlesRef.current.slice(i + 1).forEach(p2 => {
            const dx = p1.x - p2.x;
            const dy = p1.y - p2.y;
            const distance = Math.sqrt(dx * dx + dy * dy);

            if (distance < 120) {
              ctx.beginPath();
              ctx.strokeStyle = color.replace('0.5)', `${(1 - distance / 120) * 0.15})`);
              ctx.lineWidth = 1;
              ctx.moveTo(p1.x, p1.y);
              ctx.lineTo(p2.x, p2.y);
              ctx.stroke();
            }
          });
        });
      }

      animationRef.current = requestAnimationFrame(animate);
    };

    animate(0);

    // Mouse move handler
    const handleMouseMove = (e) => {
      mouseRef.current = { x: e.clientX, y: e.clientY };
    };

    const handleMouseLeave = () => {
      mouseRef.current = { x: null, y: null };
    };

    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseleave', handleMouseLeave);

    // Visibility change handler
    const handleVisibilityChange = () => {
      setIsVisible(!document.hidden);
    };

    document.addEventListener('visibilitychange', handleVisibilityChange);

    return () => {
      window.removeEventListener('resize', resizeCanvas);
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseleave', handleMouseLeave);
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [count, color, isVisible]);

  // Check for reduced motion
  const prefersReducedMotion = typeof window !== 'undefined' &&
    window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  if (prefersReducedMotion) {
    return null;
  }

  return (
    <canvas
      ref={canvasRef}
      className="particleCanvas"
      aria-hidden="true"
    />
  );
}

export default ParticleBackground;
