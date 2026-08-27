import { PointCloudRenderer } from './pointCloudRenderer.js';

export class GaussianSplatRenderer extends PointCloudRenderer {
    constructor(options = {}) {
        super({ ...options, type: 'gaussian-splat-roi' });
        this.assetUrl = options.assetUrl || null;
        this.roi = options.roi || null;
    }

    async load(payload = {}) {
        this.assetUrl = payload.asset_url || payload.assetUrl || this.assetUrl;
        this.roi = payload.roi || this.roi;
        return {
            renderer: this.type,
            status: this.assetUrl ? 'ready' : 'asset_missing',
            assetUrl: this.assetUrl,
            roi: this.roi
        };
    }

    getSelectionPreview() {
        return {
            renderer: this.type,
            status: 'visual-only'
        };
    }
}
