/**
 * Color Mode Toggle (swizzled to render nothing).
 *
 * The built-in Docusaurus toggle is hidden on purpose: the custom
 * ThemeToggle component (Light / Dark / System menu) is the site's only
 * theme control. This swizzle renders null instead of removing the toggle
 * with `colorMode.disableSwitch` — Docusaurus deletes the visitor's stored
 * theme on every mount when disableSwitch is true, so a Light/Dark choice
 * would never survive a page load (found live 2026-11-10).
 */
import React from "react";

export default function ColorModeToggle() {
  return null;
}
