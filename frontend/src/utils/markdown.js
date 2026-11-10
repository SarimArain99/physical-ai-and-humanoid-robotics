// Small safe markdown rendering for AI-rewritten page text (Level
// rewrite, Urdu translation).
//
// The rewrite comes back as markdown ("- **Define Physical AI...**"),
// but the page element it replaces is already rendered HTML, and the
// old code wrote the reply into element.innerText — so every marker
// showed literally. This module renders the reply the way the chapter
// does: inline markdown becomes real bold / italic / code / links, and
// a list marker the model echoes for an <li> is dropped (the element
// already renders its own bullet).
//
// Safety: the text is HTML-escaped FIRST, then only the constructs
// below are turned into tags — the same escape-then-render pattern the
// chat widget uses for assistant answers. No innerHTML from unescaped
// input anywhere, and links only allow http(s) and same-site relative
// URLs (javascript: and data: are dropped to plain text).
//
// Styling: plain <strong>, <em>, <code> and <a> tags, so the text
// inherits the chapter's own typography and the locked palette — no
// new colours, no new classes, no new tokens.

export function escapeHtml(text) {
  return String(text)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

// Links are rendered by a small scanner, not a regex: a regex cannot
// match the balanced parens a URL may contain (e.g. a Wikipedia link),
// and a half-matched unsafe link would leave a stray ")" behind.
// The scan runs on already-escaped text, so the label and URL are used
// as-is (escaping them again would double-escape). Links inside
// <code> spans stay literal.
function renderLinksInText(text) {
  let out = "";
  let i = 0;
  while (i < text.length) {
    const open = text.indexOf("[", i);
    if (open === -1) {
      out += text.slice(i);
      break;
    }
    const closeBracket = text.indexOf("]", open + 1);
    const paren = closeBracket === -1 ? -1 : text.indexOf("(", closeBracket + 1);
    if (closeBracket === -1 || paren !== closeBracket + 1) {
      out += text.slice(i, open + 1);
      i = open + 1;
      continue;
    }
    // Find the matching close paren, counting nesting.
    let depth = 0;
    let closeParen = -1;
    for (let j = paren; j < text.length; j++) {
      if (text[j] === "(") depth += 1;
      else if (text[j] === ")") {
        depth -= 1;
        if (depth === 0) {
          closeParen = j;
          break;
        }
      }
    }
    const label = closeParen === -1 ? "" : text.slice(open + 1, closeBracket);
    const url = closeParen === -1 ? "" : text.slice(paren + 1, closeParen);
    if (closeParen === -1 || !label || !url || /\s/.test(url)) {
      out += text.slice(i, open + 1);
      i = open + 1;
      continue;
    }
    const safe =
      /^(https?:\/\/|\/|#)/.test(url) || !/^[a-z][a-z0-9+.-]*:/i.test(url);
    out += text.slice(i, open);
    out += safe ? `<a href="${url}">${label}</a>` : label;
    i = closeParen + 1;
  }
  return out;
}

function renderLinks(html) {
  return html
    .split(/(<code>.*?<\/code>)/g)
    .map((part) => (part.startsWith("<code>") ? part : renderLinksInText(part)))
    .join("");
}

// One line of rewritten markdown as HTML: inline code, bold, italic,
// links. No block structure — the target element (<p> or <li>) already
// provides that.
export function renderInlineMarkdown(text) {
  let html = escapeHtml(text);
  html = html.replace(/`([^`\n]+)`/g, "<code>$1</code>");
  html = html.replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");
  html = html.replace(/(^|[^*])\*([^*\n]+)\*/g, "$1<em>$2</em>");
  html = renderLinks(html);
  return html;
}

// One leading list marker ("- ", "* ", "+ ", "1. ", "1) "), if present.
const LIST_MARKER = /^\s*(?:[-*+]|\d+[.)])\s+/;

export function stripListMarker(line) {
  return line.replace(LIST_MARKER, "");
}

// Render rewritten markdown into a chapter element (<p> or <li>).
// A list marker is dropped from every line: for an <li> the bullet is
// already there, and a <p> must not gain a literal "- " either. Lines
// are joined with <br> (a multi-line reply stays inside the one
// element instead of breaking out of it). Returns false when there is
// nothing to render.
export function applyRewrittenText(element, markdown) {
  if (!element || typeof markdown !== "string" || !markdown.trim()) {
    return false;
  }
  const lines = markdown
    .trim()
    .split("\n")
    .map((line) => stripListMarker(line).trim())
    .filter((line) => line.length > 0);
  if (lines.length === 0) return false;
  element.innerHTML = lines.map(renderInlineMarkdown).join("<br>");
  return true;
}
