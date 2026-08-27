const THEME_STORAGE_KEY = 'power-inspection-theme';
const VALID_THEMES = new Set(['light', 'dark']);

let currentTheme = 'light';

function normalizeTheme(theme) {
    return VALID_THEMES.has(theme) ? theme : 'light';
}

export function readStoredTheme() {
    try {
        return normalizeTheme(window.localStorage.getItem(THEME_STORAGE_KEY));
    } catch (_) {
        return 'light';
    }
}

function updateThemeToggle(theme) {
    const isDark = theme === 'dark';
    document.querySelectorAll('[data-theme-toggle]').forEach((button) => {
        button.classList.toggle('active', isDark);
        button.setAttribute('aria-pressed', isDark ? 'true' : 'false');
        button.title = isDark ? '切换浅色模式' : '切换夜间模式';
    });
    document.querySelectorAll('[data-theme-toggle-icon]').forEach((icon) => {
        icon.className = `ph-bold ${isDark ? 'ph-sun' : 'ph-moon-stars'} text-xl`;
    });
    document.querySelectorAll('[data-theme-toggle-label]').forEach((label) => {
        label.textContent = isDark ? '浅色模式' : '夜间模式';
    });
}

export function applyTheme(theme, options = {}) {
    const nextTheme = normalizeTheme(theme);
    currentTheme = nextTheme;
    document.documentElement.dataset.theme = nextTheme;
    document.documentElement.style.colorScheme = nextTheme;
    document.body.dataset.theme = nextTheme;
    document.body.classList.toggle('app-theme-dark', nextTheme === 'dark');
    document.body.classList.toggle('app-theme-light', nextTheme === 'light');
    if(options.persist) {
        try {
            window.localStorage.setItem(THEME_STORAGE_KEY, nextTheme);
        } catch (_) {}
    }
    updateThemeToggle(nextTheme);
    window.dispatchEvent(new CustomEvent('app-theme-change', {
        detail: { theme: nextTheme }
    }));
    return nextTheme;
}

export function getTheme() {
    return currentTheme;
}
