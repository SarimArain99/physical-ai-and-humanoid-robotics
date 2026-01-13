/**
 * Code Highlight Component
 * Feature: 002-ui-improvements
 * Task: T025
 *
 * Syntax highlighting wrapper for code blocks in chat responses.
 * Includes copy button functionality.
 */

import React, { useState } from 'react';
import './CodeHighlight.css';

/**
 * CodeHighlight Component
 *
 * Wraps code blocks with syntax highlighting and a copy button.
 * Uses Docusaurus's existing Prism.js configuration.
 *
 * @param {Object} props
 * @param {string} props.code - The code to display
 * @param {string} props.language - Programming language for syntax highlighting
 * @param {boolean} props.showLineNumbers - Whether to show line numbers (default: false)
 */
export default function CodeHighlight({
  code,
  language = 'javascript',
  showLineNumbers = false
}) {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(code);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy code:', err);
    }
  };

  return (
    <div className={`code-highlight-container ${language}`}>
      <div className="code-highlight-header">
        <span className="code-language">{language}</span>
        <button
          type="button"
          className={`code-copy-button ${copied ? 'code-copy-copied' : ''}`}
          onClick={handleCopy}
          aria-label={copied ? 'Copied!' : 'Copy code'}
          title="Copy code"
        >
          {copied ? (
            <>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="20 6 9 17 4 12" />
              </svg>
              Copied!
            </>
          ) : (
            <>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <rect x="9" y="9" width="13" height="13" rx="2" ry="2" />
                <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1" />
              </svg>
              Copy
            </>
          )}
        </button>
      </div>
      <pre className={`code-highlight-content language-${language}`}>
        <code>{code}</code>
      </pre>
    </div>
  );
}

/**
 * InlineCode Component
 * For inline code snippets in chat messages
 */
export function InlineCode({ children }) {
  return (
    <code className="inline-code-highlight">
      {children}
    </code>
  );
}

/**
 * CodeBlock Component
 * For multi-line code blocks in chat messages
 */
export function CodeBlock({ children, language = 'javascript', className = '' }) {
  const codeString = typeof children === 'string' ? children : String(children);

  return (
    <CodeHighlight code={codeString} language={language} showLineNumbers={false} />
  );
}
