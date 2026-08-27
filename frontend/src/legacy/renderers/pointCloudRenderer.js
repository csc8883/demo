export class PointCloudRenderer {
    constructor(options = {}) {
        this.id = options.id || null;
        this.type = options.type || 'base';
        this.theme = options.theme || 'light';
        this.visible = options.visible !== false;
        this.opacity = 1;
        this.viewOverride = null;
    }

    async load() {
        throw new Error('PointCloudRenderer.load must be implemented by subclasses');
    }

    dispose() {}

    setVisible() {}

    setOpacity() {}

    setPointSize() {}

    focus() {}

    setViewOverride(override = null) {
        this.viewOverride = override ? { ...override } : null;
    }

    setTheme(theme) {
        this.theme = theme || 'light';
    }

    getSelectionPreview() {
        return null;
    }
}
