import { PointCloudRenderer } from './pointCloudRenderer.js';

export class ThreePointsRenderer extends PointCloudRenderer {
    constructor(options = {}) {
        super({ ...options, type: 'three-points' });
        this.ops = options.ops || {};
        this.data = null;
        this.object = null;
    }

    async load(data, id = this.id) {
        this.id = id;
        this.data = data;
        this.object = this.ops.render?.(data, id) || null;
        if(this.object) this.object.visible = this.visible;
        this.applyViewStyle();
        return { renderer: this.type, id };
    }

    dispose() {
        if(this.object) this.ops.disposeObject?.(this.object);
        this.object = null;
    }

    setVisible(visible) {
        this.visible = !!visible;
        if(this.object) this.object.visible = this.visible;
    }

    setOpacity(opacity) {
        this.opacity = Math.max(0.15, Math.min(1, Number(opacity) || 1));
        this.applyViewStyle();
    }

    setPointSize(scale) {
        const value = Math.max(0.25, Math.min(3, Number(scale) || 1));
        this.object?.traverse((child) => {
            if(child.isPoints && child.material) {
                child.material.size = (child.userData.baseSize || child.material.size || 1) * value;
                child.material.needsUpdate = true;
            }
        });
    }

    focus() {
        if(this.object) this.ops.fitObject?.(this.object);
    }

    setTheme(theme) {
        super.setTheme(theme);
        this.ops.setTheme?.(theme, this.object);
    }

    setViewOverride(override = null) {
        super.setViewOverride(override);
        this.applyViewStyle();
    }

    applyViewStyle() {
        const opacityFactor = Number(this.viewOverride?.opacityFactor ?? 1);
        const plainColor = this.viewOverride?.colorMode === 'plain'
            ? (this.viewOverride.plainColor || [0.48, 0.52, 0.58])
            : null;
        this.object?.traverse((child) => {
            if(!child.isPoints || !child.material) return;
            const colorAttribute = child.geometry?.getAttribute?.('color');
            if(colorAttribute) {
                if(!child.userData.canonicalColors) {
                    child.userData.canonicalColors = colorAttribute.array.slice();
                }
                const canonical = child.userData.canonicalColors;
                if(plainColor) {
                    for(let index = 0; index < colorAttribute.count; index += 1) {
                        colorAttribute.setXYZ(index, plainColor[0], plainColor[1], plainColor[2]);
                    }
                } else if(canonical?.length === colorAttribute.array.length) {
                    colorAttribute.array.set(canonical);
                }
                colorAttribute.needsUpdate = true;
            }
            const baseOpacity = Number(child.userData.baseOpacity ?? 1);
            child.material.transparent = true;
            child.material.opacity = baseOpacity * this.opacity * opacityFactor;
            child.material.needsUpdate = true;
        });
    }

    getSelectionPreview() {
        return {
            renderer: this.type,
            id: this.id,
            sampledPointCount: this.data?.points?.length || 0
        };
    }
}
