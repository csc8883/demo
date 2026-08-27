import { useCallback, useLayoutEffect, useState } from 'react';
import legacyMarkup from './legacy/legacy-markup.html?raw';
import { bootApp } from './legacy/appController.js';
import { applyTheme, readStoredTheme } from './legacy/theme.js';

function bodyClassName(theme) {
  return [
    'flex h-screen w-screen overflow-hidden',
    theme === 'dark'
      ? 'app-theme-dark bg-slate-950 text-slate-100'
      : 'app-theme-light bg-slate-50 text-slate-800',
  ].join(' ');
}

export default function App() {
  const [theme, setTheme] = useState(readStoredTheme);

  const toggleTheme = useCallback(() => {
    setTheme((current) => (current === 'dark' ? 'light' : 'dark'));
  }, []);

  useLayoutEffect(() => {
    window.toggleTheme = toggleTheme;
    window.setAppTheme = (nextTheme) => {
      if (nextTheme === 'light' || nextTheme === 'dark') setTheme(nextTheme);
    };
    return () => {
      if (window.toggleTheme === toggleTheme) delete window.toggleTheme;
      delete window.setAppTheme;
    };
  }, [toggleTheme]);

  useLayoutEffect(() => {
    document.body.className = bodyClassName(theme);
    applyTheme(theme, { persist: true });
  }, [theme]);

  useLayoutEffect(() => {
    bootApp().catch((error) => {
      console.error('Frontend boot failed:', error);
    });
  }, []);

  return <div className="legacy-app-root" dangerouslySetInnerHTML={{ __html: legacyMarkup }} />;
}
