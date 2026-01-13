import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';
import ModuleThemeProvider from '@site/src/components/ModuleThemeProvider';
import ReadingProgress from '@site/src/components/ReadingProgress';
import ChapterNavigation from '@site/src/components/ChapterNavigation';
import PageTransition from '@site/src/components/PageTransition';
import VisualEffects from '@site/src/components/VisualEffects';
import useIsBrowser from '@docusaurus/useIsBrowser';

/**
 * Swizzled Layout Component
 * Feature: 002-ui-improvements
 * Task: T014, T042, T043, T050, T086
 *
 * Wraps the original Docusaurus Layout with:
 * - ModuleThemeProvider for dynamic theming
 * - AuthProvider for authentication
 * - ChatWidget for AI assistant
 * - ReadingProgress for scroll tracking
 * - ChapterNavigation for prev/next links
 * - PageTransition for smooth page transitions
 * - VisualEffects for particle backgrounds and scroll animations
 */
export default function Layout(props) {
  const isBrowser = useIsBrowser();
  const isDocPage = props?.location?.pathname?.startsWith('/docs') || false;
  const isHomePage = props?.location?.pathname === '/' || false;

  return (
    <ModuleThemeProvider>
      <AuthProvider>
        {/* VisualEffects enabled with particles on homepage, scroll animations globally */}
        <VisualEffects
          enableParticles={isHomePage}
          enableScrollAnimations={true}
          particleCount="auto"
        >
          {isDocPage && isBrowser && <ReadingProgress />}
          <PageTransition>
            <OriginalLayout {...props} />
          </PageTransition>
          <ChatWidget />
          {isDocPage && <ChapterNavigationWrapper />}
        </VisualEffects>
      </AuthProvider>
    </ModuleThemeProvider>
  );
}

/**
 * Wrapper for ChapterNavigation to detect doc pages
 */
function ChapterNavigationWrapper() {
  const [currentPath, setCurrentPath] = React.useState(null);

  React.useEffect(() => {
    if (typeof window !== 'undefined') {
      setCurrentPath(window.location.pathname);
    }
  }, []);

  // Only show on doc pages
  if (!currentPath?.startsWith('/docs')) {
    return null;
  }

  return <ChapterNavigation currentPath={currentPath} />;
}