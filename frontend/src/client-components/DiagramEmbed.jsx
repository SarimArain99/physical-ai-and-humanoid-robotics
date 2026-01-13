/**
 * Diagram Embed Component for MDX
 * Feature: 002-ui-improvements
 * Task: T068
 *
 * Usage in MDX:
 * ```jsx
 * import Diagram from '@site/src/client-components/DiagramEmbed';
 *
 * <Diagram id="ros2-nodes-topics" />
 * ```
 */

import React from 'react';
import InteractiveDiagram from '@site/src/components/InteractiveDiagram';

export default function DiagramEmbed({ id, width, height, animated = true }) {
  return (
    <InteractiveDiagram
      diagramId={id}
      width={width || 700}
      height={height || 400}
      animated={animated}
    />
  );
}
