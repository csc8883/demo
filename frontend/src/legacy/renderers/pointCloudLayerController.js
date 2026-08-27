function requestKey(descriptor = {}) {
    const variant = descriptor.variant || 'base';
    const profileId = descriptor.profileId || descriptor.profile_id || '';
    return `${variant}::${profileId}`;
}

export class PointCloudLayerController {
    constructor(options = {}) {
        this.id = options.id || null;
        this.renderer = null;
        this.desiredVisible = options.visible !== false;
        this.viewOverride = null;
        this.requestVersion = 0;
        this.currentRequest = null;
        this.onRendererChanged = options.onRendererChanged || null;
    }

    beginLodRequest(descriptor = {}) {
        this.requestVersion += 1;
        this.currentRequest = {
            id: this.id,
            version: this.requestVersion,
            key: requestKey(descriptor),
            variant: descriptor.variant || 'base',
            profileId: descriptor.profileId || descriptor.profile_id || null
        };
        return { ...this.currentRequest };
    }

    invalidateLodRequests() {
        this.requestVersion += 1;
        this.currentRequest = null;
    }

    isCurrentRequest(request) {
        if(!request || !this.currentRequest) return false;
        return request.id === this.id
            && request.version === this.currentRequest.version
            && request.key === this.currentRequest.key;
    }

    commitRenderer(nextRenderer, request = null) {
        if(!nextRenderer) return false;
        if(request && !this.isCurrentRequest(request)) {
            nextRenderer.dispose?.();
            return false;
        }

        const previousRenderer = this.renderer;
        if(previousRenderer === nextRenderer) {
            nextRenderer.setViewOverride?.(this.viewOverride);
            nextRenderer.setVisible?.(this.desiredVisible);
            return true;
        }

        previousRenderer?.setVisible?.(false);
        this.renderer = nextRenderer;
        nextRenderer.setViewOverride?.(this.viewOverride);
        nextRenderer.setVisible?.(this.desiredVisible);
        this.onRendererChanged?.(nextRenderer, previousRenderer);
        previousRenderer?.dispose?.();
        return true;
    }

    setVisible(visible) {
        this.desiredVisible = !!visible;
        this.renderer?.setVisible?.(this.desiredVisible);
    }

    setViewOverride(override = null) {
        this.viewOverride = override ? { ...override } : null;
        this.renderer?.setViewOverride?.(this.viewOverride);
    }

    setOpacity(opacity) {
        this.renderer?.setOpacity?.(opacity);
    }

    setPointSize(scale) {
        this.renderer?.setPointSize?.(scale);
    }

    setTheme(theme) {
        this.renderer?.setTheme?.(theme);
    }

    setRenderHints(hints = {}) {
        this.renderer?.setRenderHints?.(hints);
    }

    update(camera, renderer) {
        return this.renderer?.update?.(camera, renderer) || null;
    }

    focus() {
        this.renderer?.focus?.();
    }

    getInfo() {
        return {
            ...(this.renderer?.getSelectionPreview?.() || {}),
            id: this.id,
            visible: this.desiredVisible,
            request: this.currentRequest ? { ...this.currentRequest } : null
        };
    }

    dispose() {
        this.invalidateLodRequests();
        const previousRenderer = this.renderer;
        this.renderer = null;
        this.onRendererChanged?.(null, previousRenderer);
        previousRenderer?.dispose?.();
    }
}

export function pointCloudRequestKey(descriptor = {}) {
    return requestKey(descriptor);
}
