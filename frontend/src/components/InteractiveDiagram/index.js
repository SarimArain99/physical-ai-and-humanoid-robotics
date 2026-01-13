import React, { useState, useCallback, useRef, useEffect } from 'react';
import styles from './styles.css';

/**
 * InteractiveDiagram Component
 * Feature: 002-ui-improvements
 * Task: T062, T065, T067, T069
 *
 * Renders interactive SVG diagrams with:
 * - Clickable nodes with tooltips
 * - Data flow animations
 * - Keyboard navigation for accessibility
 */

// Node type colors
const NODE_COLORS = {
  component: { fill: '#2ECC71', stroke: '#25B365' },
  channel: { fill: '#4ECDC4', stroke: '#3BA39C' },
  middleware: { fill: '#9B59B6', stroke: '#8E44AD' },
  sensor: { fill: '#E74C3C', stroke: '#C0392B' },
  processing: { fill: '#F39C12', stroke: '#E67E22' },
  output: { fill: '#3498DB', stroke: '#2980B9' },
  server: { fill: '#1E2A38', stroke: '#2C3E50' },
  topic: { fill: '#4ECDC4', stroke: '#3BA39C' },
  physical: { fill: '#E67E22', stroke: '#D35400' },
  virtual: { fill: '#9B59B6', stroke: '#8E44AD' },
  dataflow: { fill: '#95A5A6', stroke: '#7F8C8D' },
  localization: { fill: '#16A085', stroke: '#138D75' },
  planner: { fill: '#27AE60', stroke: '#229954' },
  controller: { fill: '#2980B9', stroke: '#2471A3' },
  behavior: { fill: '#F1C40F', stroke: '#F39C12' }
};

const DEFAULT_NODE_COLOR = { fill: '#64748B', stroke: '#475569' };

function getNodeColor(type) {
  return NODE_COLORS[type] || DEFAULT_NODE_COLOR;
}

export default function InteractiveDiagram({
  diagram = null,
  diagramId = null,
  width = 700,
  height = 400,
  animated = true,
  interactive = true
}) {
  const [selectedNode, setSelectedNode] = useState(null);
  const [hoveredNode, setHoveredNode] = useState(null);
  const [activeAnimation, setActiveAnimation] = useState(null);
  const containerRef = useRef(null);

  // Get diagram data
  const diagramData = diagram || (diagramId ? getDiagramById(diagramId) : null);

  if (!diagramData) {
    return (
      <div className={styles.error} style={{ width, height }}>
        Diagram not found. Please provide valid diagram or diagramId.
      </div>
    );
  }

  const { nodes, connections } = diagramData;

  /**
   * Handle node click
   */
  const handleNodeClick = useCallback((node, event) => {
    if (!interactive) return;

    if (selectedNode?.id === node.id) {
      setSelectedNode(null);
    } else {
      setSelectedNode(node);
    }

    // Trigger animation on connected paths
    if (animated) {
      setActiveAnimation({
        nodeId: node.id,
        timestamp: Date.now()
      });
    }
  }, [selectedNode, interactive, animated]);

  /**
   * Handle keyboard navigation
   */
  const handleKeyDown = useCallback((event, node, index) => {
    if (!interactive) return;

    switch (event.key) {
      case 'Enter':
      case ' ':
        event.preventDefault();
        handleNodeClick(node, event);
        break;
      case 'ArrowRight':
        event.preventDefault();
        const nextIndex = (index + 1) % nodes.length;
        setHoveredNode(nodes[nextIndex]);
        break;
      case 'ArrowLeft':
        event.preventDefault();
        const prevIndex = (index - 1 + nodes.length) % nodes.length;
        setHoveredNode(nodes[prevIndex]);
        break;
      case 'Escape':
        setSelectedNode(null);
        setHoveredNode(null);
        break;
    }
  }, [nodes, handleNodeClick, interactive]);

  /**
   * Handle mouse enter
   */
  const handleMouseEnter = useCallback((node) => {
    if (!interactive) return;
    setHoveredNode(node);
  }, [interactive]);

  /**
   * Handle mouse leave
   */
  const handleMouseLeave = useCallback(() => {
    if (!interactive) return;
    setHoveredNode(null);
  }, [interactive]);

  // Reset animation after it completes
  useEffect(() => {
    if (activeAnimation) {
      const timer = setTimeout(() => {
        setActiveAnimation(null);
      }, 2000);
      return () => clearTimeout(timer);
    }
  }, [activeAnimation]);

  // Scale nodes to fit in view
  const scaleX = (width - 100) / 700;
  const scaleY = (height - 100) / 400;
  const scale = Math.min(scaleX, scaleY, 1);

  return (
    <div
      ref={containerRef}
      className={styles.container}
      style={{ width, height }}
      role="region"
      aria-label={`Interactive diagram: ${diagramData.title}`}
      aria-describedby={selectedNode ? 'node-details' : undefined}
    >
      <div className={styles.header}>
        <h3 className={styles.title}>{diagramData.title}</h3>
        {diagramData.description && (
          <p className={styles.description}>{diagramData.description}</p>
        )}
      </div>

      <svg
        width={width}
        height={height - 80}
        viewBox={`0 0 ${width} ${height - 80}`}
        className={styles.svg}
      >
        {/* Definitions for markers and filters */}
        <defs>
          {/* Arrowhead marker */}
          <marker
            id="arrowhead"
            markerWidth="10"
            markerHeight="7"
            refX="9"
            refY="3.5"
            orient="auto"
          >
            <polygon
              points="0 0, 10 3.5, 0 7"
              fill={selectedNode ? getNodeColor(selectedNode.type).fill : '#64748B'}
            />
          </marker>

          {/* Glow filter for active nodes */}
          <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
            <feGaussianBlur stdDeviation="3" result="coloredBlur"/>
            <feMerge>
              <feMergeNode in="coloredBlur"/>
              <feMergeNode in="SourceGraphic"/>
            </feMerge>
          </filter>

          {/* Gradient for animated paths */}
          <linearGradient id="flowGradient" gradientUnits="userSpaceOnUse">
            <stop offset="0%" stopColor={selectedNode ? getNodeColor(selectedNode.type).fill : '#4ECDC4'} stopOpacity="0.2"/>
            <stop offset="50%" stopColor={selectedNode ? getNodeColor(selectedNode.type).fill : '#4ECDC4'} stopOpacity="1"/>
            <stop offset="100%" stopColor={selectedNode ? getNodeColor(selectedNode.type).fill : '#4ECDC4'} stopOpacity="0.2"/>
          </linearGradient>
        </defs>

        {/* Connections (draw first so nodes appear on top) */}
        {connections.map((conn, index) => {
          const fromNode = nodes.find(n => n.id === conn.from);
          const toNode = nodes.find(n => n.id === conn.to);
          if (!fromNode || !toNode) return null;

          const fromX = fromNode.x * scale + 50;
          const fromY = fromNode.y * scale + 40;
          const toX = toNode.x * scale + 50;
          const toY = toNode.y * scale + 40;

          const isSelected = selectedNode &&
            (selectedNode.id === conn.from || selectedNode.id === conn.to);
          const isAnimating = activeAnimation &&
            (activeAnimation.nodeId === conn.from || activeAnimation.nodeId === conn.to);

          // Calculate path
          let pathD;
          if (conn.style === 'curved') {
            const midX = (fromX + toX) / 2;
            const midY = (fromY + toY) / 2 - 30;
            pathD = `M ${fromX} ${fromY} Q ${midX} ${midY} ${toX} ${toY}`;
          } else {
            pathD = `M ${fromX} ${fromY} L ${toX} ${toY}`;
          }

          return (
            <g key={`conn-${index}`} className={styles.connectionGroup}>
              {/* Connection line */}
              <path
                d={pathD}
                className={styles.connection}
                stroke={isSelected ? getNodeColor(selectedNode.type).fill : '#64748B'}
                strokeDasharray={conn.style === 'dashed' ? '5,5' : undefined}
                markerEnd={conn.style !== 'bidirectional' ? 'url(#arrowhead)' : undefined}
                strokeWidth={isSelected ? 3 : 2}
                opacity={isSelected ? 1 : 0.6}
              />

              {/* Animated flow indicator */}
              {animated && isAnimating && (
                <circle
                  r="4"
                  fill={getNodeColor(selectedNode?.type || 'component').fill}
                  className={styles.flowParticle}
                >
                  <animateMotion
                    dur="1s"
                    repeatCount="1"
                    path={pathD}
                  />
                </circle>
              )}

              {/* Connection label */}
              {conn.label && (
                <text
                  x={(fromX + toX) / 2}
                  y={(fromY + toY) / 2 - 10}
                  className={styles.connectionLabel}
                  textAnchor="middle"
                  fontSize="11"
                  fill="#94A3B8"
                >
                  {conn.label}
                </text>
              )}

              {/* Bidirectional arrow */}
              {conn.style === 'bidirectional' && (
                <>
                  <polygon
                    points={`${toX - 5},${toY - 4} ${toX + 5},${toY} ${toX - 5},${toY + 4}`}
                    fill={isSelected ? getNodeColor(selectedNode.type).fill : '#64748B'}
                    transform={`rotate(${Math.atan2(toY - fromY, toX - fromX) * 180 / Math.PI}, ${toX}, ${toY})`}
                  />
                </>
              )}
            </g>
          );
        })}

        {/* Nodes */}
        {nodes.map((node, index) => {
          const colors = getNodeColor(node.type);
          const x = node.x * scale + 50;
          const y = node.y * scale + 40;
          const isSelected = selectedNode?.id === node.id;
          const isHovered = hoveredNode?.id === node.id;

          return (
            <g
              key={node.id}
              className={styles.nodeGroup}
              onClick={(e) => handleNodeClick(node, e)}
              onMouseEnter={() => handleMouseEnter(node)}
              onMouseLeave={handleMouseLeave}
              onFocus={() => setHoveredNode(node)}
              onBlur={() => setHoveredNode(null)}
              onKeyDown={(e) => handleKeyDown(e, node, index)}
              tabIndex={interactive ? 0 : undefined}
              role="button"
              aria-label={node.label}
              aria-pressed={isSelected}
              aria-describedby={isSelected ? 'node-details' : `node-${node.id}-desc`}
            >
              {/* Node rectangle with rounded corners */}
              <rect
                x={x - 60}
                y={y - 25}
                width="120"
                height="50"
                rx="8"
                fill={colors.fill}
                stroke={colors.stroke}
                strokeWidth={isSelected ? 3 : isHovered ? 2 : 1}
                className={styles.node}
                filter={isSelected ? 'url(#glow)' : undefined}
              />

              {/* Node label */}
              <text
                x={x}
                y={y + 5}
                className={styles.nodeLabel}
                textAnchor="middle"
                fill="white"
                fontWeight="600"
                fontSize="13"
              >
                {node.label}
              </text>

              {/* Hidden description for screen readers */}
              <text
                id={`node-${node.id}-desc`}
                x={x}
                y={y + 45}
                className={styles.hidden}
                textAnchor="middle"
              >
                {node.description}
              </text>
            </g>
          );
        })}
      </svg>

      {/* Node details panel */}
      {selectedNode && (
        <div
          id="node-details"
          className={styles.detailsPanel}
          role="dialog"
          aria-labelledby="details-title"
        >
          <button
            className={styles.closeButton}
            onClick={() => setSelectedNode(null)}
            aria-label="Close details"
          >
            ×
          </button>
          <h4 id="details-title" className={styles.detailsTitle}>
            {selectedNode.label}
          </h4>
          <p className={styles.detailsDescription}>{selectedNode.description}</p>
          {selectedNode.details && selectedNode.details.length > 0 && (
            <ul className={styles.detailsList}>
              {selectedNode.details.map((detail, i) => (
                <li key={i}>{detail}</li>
              ))}
            </ul>
          )}
        </div>
      )}

      {/* Keyboard instructions */}
      {interactive && (
        <div className={styles.keyboardHint}>
          <span className={styles.keyboardKey}>Tab</span> to navigate ·
          <span className={styles.keyboardKey}>Enter</span> to select ·
          <span className={styles.keyboardKey}>Esc</span> to close
        </div>
      )}
    </div>
  );
}

/**
 * Helper to get diagram by ID
 */
function getDiagramById(id) {
  // This will be imported from the data file
  try {
    const diagrams = require('@site/src/data/diagrams');
    return diagrams.getDiagram ? diagrams.getDiagram(id) : diagrams.DIAGRAMS?.[id];
  } catch {
    return null;
  }
}

/**
 * Export tooltip component
 */
export function DiagramTooltip({ node, x, y }) {
  if (!node) return null;

  return (
    <div
      className={styles.tooltip}
      style={{ left: x, top: y }}
      role="tooltip"
    >
      <strong>{node.label}</strong>
      <p>{node.description}</p>
    </div>
  );
}

/**
 * Export simple diagram renderer for MDX
 */
export function Diagram({ id, ...props }) {
  return <InteractiveDiagram diagramId={id} {...props} />;
}
