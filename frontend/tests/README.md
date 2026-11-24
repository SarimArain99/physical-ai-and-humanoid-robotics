# Testing Guide: UI/UX Improvements

**Feature**: 002-ui-improvements
**Tasks**: T088, T089, T090, T093

This guide covers accessibility auditing, cross-browser testing, responsive design testing, and visual regression testing.

---

## Playwright Visual Regression Tests (T093) — spec deleted 2026-11-24

The only spec file (`tests/e2e/visual-regression.spec.js`) was deleted: it waited up to 5s for `.three-d-robot-container` / `.three-d-robot-fallback-svg`, but the 3D robot was removed entirely, so every homepage test timed out; it also asserted the Orbitron font (retired — headings now use Space Grotesk/Inter) and Week Cards (component removed), and its overlap check flagged parent/child pairs as overlaps. It could not be fixed by editing — it described a homepage that no longer exists.

A replacement spec, when written, should assert against the current homepage:

- Homepage loads at 375/768/1280px with the hero headline and the Sense–Think–Act loop visible (no 3D robot).
- No horizontal scroll at 320/375/414px.
- A chapter page renders its quiz (e.g. `/docs/week3/ros2-nodes-topics` shows a quiz) and the reading progress bar.
- The chat widget icon is visible on the homepage.
- Module theming: `body[data-module]` is set and `--module-primary` is non-empty on a doc page.
- Headings use the current font stack (Space Grotesk/Inter), not Orbitron.

The Playwright setup (`playwright.config.js`, `npm run test:e2e*` scripts) is unchanged; add the new spec under `tests/e2e/` and generate screenshot baselines with `npm run test:visual:update` before relying on screenshot comparisons.

---

## Accessibility Audit (T088)

### Using axe DevTools

1. Install axe DevTools browser extension
2. Navigate to pages to test
3. Open axe DevTools and scan
4. Fix any critical/serious issues

### Key Accessibility Requirements

- **Color Contrast**: WCAG AA requires 4.5:1 for normal text, 3:1 for large text
- **Keyboard Navigation**: All interactive elements must be keyboard accessible
- **Screen Reader**: Proper ARIA labels and roles
- **Focus Indicators**: Visible focus states for all interactive elements
- **Reduced Motion**: Respects `prefers-reduced-motion` preference

### Manual Checklist

- [ ] Navigate homepage using Tab key
- [ ] Verify focus indicators are visible
- [ ] Test with screen reader (NVDA, VoiceOver, or JAWS)
- [ ] Check color contrast with axe DevTools
- [ ] Verify all images have alt text
- [ ] Test reduced motion preference

---

## Cross-Browser Testing (T089)

### Browsers to Test

| Browser | Version | Platform |
|---------|---------|----------|
| Chrome | Latest | Windows, macOS, Linux |
| Firefox | Latest | Windows, macOS, Linux |
| Safari | Latest | macOS only |
| Edge | Latest | Windows, macOS |

### Playwright Multi-Browser Tests

Run tests across all browsers:

```bash
npm run test:e2e --project=chromium
npm run test:e2e --project=firefox
npm run test:e2e --project=webkit
```

### Key Browser Compatibility Checks

1. **CSS Custom Properties**: Module theming uses CSS variables (all modern browsers)
2. **WebGL**: ThreeDRobot requires WebGL support
3. **Flexbox/Grid**: Layout uses modern CSS
4. **Animations**: CSS transitions with `prefers-reduced-motion` fallback

---

## Responsive Design Testing (T090)

### Viewports to Test

| Device | Width | Height | Type |
|--------|-------|--------|------|
| iPhone 5 | 320px | 568px | Mobile (Small) |
| iPhone SE | 375px | 667px | Mobile (Medium) |
| iPhone 11 | 414px | 896px | Mobile (Large) |
| iPad | 768px | 1024px | Tablet |
| Desktop | 1280px+ | 720px+ | Desktop |

### Testing Tools

1. **Browser DevTools**: Device emulation mode
2. **Playwright**: Automated responsive tests
3. **Real Devices**: Test on actual mobile devices

### Checklist

- [ ] No horizontal scroll at 320px width
- [ ] Text is readable without zooming
- [ ] Touch targets are at least 44x44px
- [ ] 3D robot model scales properly
- [ ] Week cards stack vertically on mobile
- [ ] Chat widget doesn't cover content

---

## Performance Testing (T061)

### ThreeDRobot FPS Target

- **Minimum**: 30 FPS on mid-range devices
- **Target**: 60 FPS on modern devices

### FPS Monitoring

Enable FPS display in development:

```javascript
<ThreeDRobot showPerfStats={true} />
```

This shows an FPS counter in the top-left corner of the 3D robot canvas.

### Performance Optimization Checklist

- [ ] ThreeDRobot uses lazy loading (React.lazy)
- [ ] Low-end devices use reduced geometry
- [ ] Pixel ratio capped at 2x
- [ ] Animation frame throttling on low-performance devices
- [ ] WebGL fallback for unsupported browsers

---

## CI/CD Integration

### GitHub Actions Example

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '20'
      - run: cd frontend && npm ci
      - run: cd frontend && npx playwright install --with-deps
      - run: cd frontend && npm run build
      - run: cd frontend && npm run test:e2e
      - uses: actions/upload-artifact@v3
        if: always()
        with:
          name: playwright-report
          path: frontend/playwright-report/
          retention-days: 7
```
