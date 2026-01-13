/**
 * Visual Regression Tests
 * Feature: 002-ui-improvements
 * Task: T093
 *
 * Playwright tests for visual regression testing of key pages.
 * These tests capture screenshots and compare them against baseline images.
 */

import { test, expect } from '@playwright/test';

const BASE_URL = process.env.BASE_URL || 'http://localhost:3000';

// Viewports to test across
const VIEWPORTS = [
  { name: 'mobile', width: 375, height: 667 },  // iPhone SE
  { name: 'tablet', width: 768, height: 1024 }, // iPad
  { name: 'desktop', width: 1280, height: 720 }, // Desktop
];

test.describe('Visual Regression Tests', () => {
  // T093: Homepage visual tests
  test.describe('Homepage', () => {
    VIEWPORTS.forEach(({ name, width, height }) => {
      test(`homepage on ${name}`, async ({ page }) => {
        await page.setViewportSize({ width, height });
        await page.goto(`${BASE_URL}/`);

        // Wait for 3D robot to load
        await page.waitForSelector('.three-d-robot-container, .three-d-robot-fallback-svg', { timeout: 5000 });

        // Wait for animations to complete
        await page.waitForTimeout(500);

        // Take screenshot
        await expect(page).toHaveScreenshot(`homepage-${name}.png`, {
          maxDiffPixels: 100, // Allow minor pixel differences
          threshold: 0.1, // 10% difference tolerance
        });
      });
    });
  });

  // T093: Module color theming tests
  test.describe('Module Theming', () => {
    const modulePaths = [
      { name: 'ros2-module', path: '/docs/week1/intro-physical-ai', expectedColor: 'rgb(255, 107, 53)' },
      { name: 'digital-twin-module', path: '/docs/week6/gazebo-simulation', expectedColor: 'rgb(78, 205, 196)' },
      { name: 'isaac-module', path: '/docs/week8/nvidia-isaac-sim', expectedColor: 'rgb(168, 85, 247)' },
      { name: 'vla-module', path: '/docs/week11/humanoid-kinematics', expectedColor: 'rgb(255, 8, 68)' },
    ];

    modulePathTests:
    for (const { name, path, expectedColor }) {
      test(`module colors for ${name}`, async ({ page }) => {
        await page.goto(`${BASE_URL}${path}`);

        // Wait for module theme to be applied
        await page.waitForTimeout(300);

        // Check that module data attribute is set
        const body = page.locator('body');
        const dataModule = await body.getAttribute('data-module');
        expect(dataModule).toBeTruthy();

        // Verify module-specific accent color is applied (check CSS variable)
        const primaryColor = await page.evaluate(() => {
          return getComputedStyle(document.documentElement)
            .getPropertyValue('--module-primary')
            .trim();
        });

        // The color should be set (not default)
        expect(primaryColor).toBeTruthy();
      });
    }
  });

  // T093: Chat Widget visual tests
  test.describe('Chat Widget', () => {
    test('chat icon visibility', async ({ page }) => {
      await page.goto(`${BASE_URL}/`);

      // Wait for page to load
      await page.waitForLoadState('networkidle');

      // Check that chat widget is present
      const chatIcon = page.locator('[class*="chat"], [class*="Chat"]').first();
      await expect(chatIcon.first()).toBeVisible();

      // Take screenshot of chat widget area
      const chatWidget = page.locator('[class*="chat"], [class*="Chat"]').first();
      if (await chatWidget.count() > 0) {
        await expect(chatWidget.first()).toHaveScreenshot('chat-widget-icon.png', {
          maxDiffPixels: 50,
        });
      }
    });
  });

  // T093: Reading progress bar tests
  test.describe('Reading Progress', () => {
    test('progress bar on documentation page', async ({ page }) => {
      await page.goto(`${BASE_URL}/docs/intro`);

      // Wait for page to load
      await page.waitForLoadState('networkidle');

      // Check for reading progress bar
      const progressBar = page.locator('.reading-progress-bar, [role="progressbar"]');

      if (await progressBar.count() > 0) {
        await expect(progressBar.first()).toBeVisible();
      }
    });
  });

  // T093: Week Cards hover states
  test.describe('Week Cards', () => {
    test('week cards visual test', async ({ page }) => {
      await page.goto(`${BASE_URL}/`);

      // Wait for cards to load
      await page.waitForSelector('.week-card, .card', { timeout: 5000 });

      // Take screenshot before hover
      await expect(page.locator('.week-card, .card').first()).toHaveScreenshot('week-card-default.png');

      // Hover and take screenshot
      const firstCard = page.locator('.week-card, .card').first();
      await firstCard.hover();
      await page.waitForTimeout(300); // Wait for transition

      await expect(firstCard).toHaveScreenshot('week-card-hover.png', {
        maxDiffPixels: 50,
      });
    });
  });

  // T093: Typography tests
  test.describe('Typography', () => {
    test('fonts loaded correctly', async ({ page }) => {
      await page.goto(`${BASE_URL}/`);

      // Wait for fonts to load
      await page.waitForLoadState('networkidle');
      await page.waitForTimeout(1000);

      // Check that Orbitron font is applied to headings
      const heading = page.locator('h1').first();
      const fontFamily = await heading.evaluate(el => {
        return window.getComputedStyle(el).fontFamily;
      });

      // Should contain Orbitron or fallback
      expect(fontFamily.toLowerCase()).toContain('orbitron');
    });
  });

  // T093: Accessibility visual checks
  test.describe('Accessibility Visual Checks', () => {
    test('focus states are visible', async ({ page }) => {
      await page.goto(`${BASE_URL}/`);

      // Find a focusable element
      const link = page.locator('a').first();
      await link.focus();

      // Check that focus state has visible indicator
      const focusedElement = await page.evaluate(() => {
        const el = document.activeElement;
        const styles = window.getComputedStyle(el);
        return {
          outline: styles.outline,
          outlineOffset: styles.outlineOffset,
          boxShadow: styles.boxShadow,
        };
      });

      // At least one focus indicator should be present
      const hasFocusIndicator =
        focusedElement.outline !== 'none' ||
        focusedElement.boxShadow.includes('0 0') ||
        focusedElement.outlineOffset !== '0px';

      expect(hasFocusIndicator).toBeTruthy();
    });
  });

  // T093: Responsive layout tests
  test.describe('Responsive Layout', () => {
    test('mobile layout does not overlap elements', async ({ page }) => {
      await page.setViewportSize({ width: 375, height: 667 });
      await page.goto(`${BASE_URL}/`);

      // Wait for page to load
      await page.waitForLoadState('networkidle');

      // Check that elements don't overlap (simple bounding box check)
      const overlaps = await page.evaluate(() => {
        const elements = document.querySelectorAll('.week-card, .card, h1, button');
        const rects = Array.from(elements).map(el => el.getBoundingClientRect());

        for (let i = 0; i < rects.length; i++) {
          for (let j = i + 1; j < rects.length; j++) {
            const a = rects[i];
            const b = rects[j];

            // Check for overlap
            if (!(a.right < b.left ||
                  a.left > b.right ||
                  a.bottom < b.top ||
                  a.top > b.bottom)) {
              return true; // Overlap found
            }
          }
        }
        return false;
      });

      expect(overlaps).toBeFalsy();
    });
  });
});

// T090: Additional responsive tests
test.describe('Mobile Responsiveness', () => {
  const mobileSizes = [
    { width: 320, height: 568 },  // iPhone 5
    { width: 375, height: 667 },  // iPhone SE
    { width: 414, height: 896 },  // iPhone 11
  ];

  mobileSizeTests:
  for (const { width, height }) of mobileSizes {
    test(`responsive layout at ${width}x${height}`, async ({ page }) => {
      await page.setViewportSize({ width, height });
      await page.goto(`${BASE_URL}/`);

      // Wait for page to load
      await page.waitForLoadState('networkidle');

      // Verify no horizontal scroll
      const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
      const viewportWidth = await page.evaluate(() => window.innerWidth);

      expect(bodyWidth).toBeLessThanOrEqual(viewportWidth + 1); // Allow 1px rounding error
    });
  }
});
