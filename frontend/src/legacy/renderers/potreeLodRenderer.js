import { PointCloudRenderer } from './pointCloudRenderer.js';
import { LOD_CLASSIFICATION_PALETTE } from './semanticStyles.js';
import { Box3, Color, Vector3, Vector4 } from 'three';

let potreeCoreModulePromise = null;

function loadPotreeCore() {
    if(!potreeCoreModulePromise) {
        potreeCoreModulePromise = import('potree-core');
    }
    return potreeCoreModulePromise;
}

export class PotreeLodRenderer extends PointCloudRenderer {
    constructor(options = {}) {
        super({ ...options, type: 'potree-lod' });
        this.manifestUrl = options.manifestUrl || null;
        this.status = options.status || 'not_loaded';
        this.scene = options.scene || null;
        this.fitObject = options.fitObject || null;
        this.globalOffset = options.globalOffset || [0, 0, 0];
        this.baseLayer = Number.isInteger(options.baseLayer) ? options.baseLayer : 1;
        this.pointBudget = options.pointBudget || 1_800_000;
        this.pointSizeScale = 1;
        this.colorMode = normalizeColorMode(options.colorMode || 'classification');
        this.classificationPalette = options.classificationPalette || defaultClassificationPalette();
        this.plainColorOverride = options.plainColor ? normalizePlainColor(options.plainColor) : null;
        this.plainColor = this.plainColorOverride || defaultPlainColor(this.theme);
        this.overlayRole = options.overlayRole || 'none';
        this.potreeCore = null;
        this.potree = null;
        this.pointClouds = [];
        this.object = null;
    }

    async load(payload = {}) {
        this.manifestUrl = payload.manifest_url || payload.manifestUrl || this.manifestUrl;
        this.setRenderHints(payload.render_hints || payload.renderHints || {});
        if(!this.manifestUrl || !this.scene) {
            this.status = 'missing_manifest';
            return {
                renderer: this.type,
                status: this.status,
                manifestUrl: this.manifestUrl
            };
        }
        const { manifestName, baseUrl } = splitManifestUrl(this.manifestUrl);
        const potreeCore = await loadPotreeCore();
        const { Potree } = potreeCore;
        this.potreeCore = potreeCore;
        this.potree = new Potree();
        this.potree.pointBudget = this.pointBudget;
        const pointCloud = await this.potree.loadPointCloud(manifestName, baseUrl);
        pointCloud.name = `potree-lod-${this.id || 'pointcloud'}`;
        pointCloud.position.x -= Number(this.globalOffset[0] || 0);
        pointCloud.position.y -= Number(this.globalOffset[1] || 0);
        pointCloud.position.z -= Number(this.globalOffset[2] || 0);
        pointCloud.visible = this.visible;
        setLayerRecursive(pointCloud, this.baseLayer);
        this.scene.add(pointCloud);
        this.object = pointCloud;
        this.pointClouds = [pointCloud];
        this.applyMaterialStyle();
        this.status = 'ready';
        const effectiveStyle = this.getEffectiveStyle();
        return {
            renderer: this.type,
            status: this.status,
            manifestUrl: this.manifestUrl,
            colorMode: effectiveStyle.colorMode,
            overlayRole: this.overlayRole
        };
    }

    update(camera, renderer) {
        if(!this.potree || !this.pointClouds.length || !camera || !renderer) return null;
        return this.potree.updatePointClouds(this.pointClouds, camera, renderer);
    }

    dispose() {
        this.pointClouds.forEach((pointCloud) => {
            if(this.scene) this.scene.remove(pointCloud);
            pointCloud.dispose?.();
        });
        this.pointClouds = [];
        this.object = null;
    }

    setVisible(visible) {
        this.visible = !!visible;
        if(this.object) this.object.visible = this.visible;
    }

    setOpacity(opacity) {
        this.opacity = Math.max(0.15, Math.min(1, Number(opacity) || 1));
        this.applyMaterialStyle();
    }

    setPointSize(scale) {
        this.pointSizeScale = Math.max(0.25, Math.min(3, Number(scale) || 1));
        this.pointClouds.forEach((pointCloud) => {
            if(pointCloud.material) {
                pointCloud.material.size = Math.max(1, 1.4 * this.pointSizeScale);
                pointCloud.material.needsUpdate = true;
            }
        });
    }

    focus() {
        if(this.object) this.fitObject?.(this.object);
    }

    setRenderHints(hints = {}) {
        if(hints.point_budget) {
            this.pointBudget = Math.max(100_000, Number(hints.point_budget) || this.pointBudget);
            if(this.potree) this.potree.pointBudget = this.pointBudget;
        }
        if(hints.color_mode || hints.colorMode) {
            this.colorMode = normalizeColorMode(hints.color_mode || hints.colorMode);
        }
        if(hints.classification_palette || hints.classificationPalette) {
            this.classificationPalette = normalizeClassificationPalette(
                hints.classification_palette || hints.classificationPalette
            );
        }
        const plainColorHint = hints.plain_color
            ?? hints.plainColor
            ?? hints.fixed_color
            ?? hints.fixedColor;
        if(plainColorHint) {
            this.plainColorOverride = normalizePlainColor(plainColorHint);
        }
        this.plainColor = this.plainColorOverride || defaultPlainColor(this.theme);
        if(hints.overlay_role || hints.overlayRole) {
            this.overlayRole = hints.overlay_role || hints.overlayRole;
        }
        this.applyMaterialStyle();
    }

    setTheme(theme = 'light') {
        super.setTheme(theme);
        this.plainColor = this.plainColorOverride || defaultPlainColor(this.theme);
        this.applyMaterialStyle();
    }

    setViewOverride(override = null) {
        super.setViewOverride(override);
        this.applyMaterialStyle();
    }

    getEffectiveStyle() {
        const opacityFactor = Number(this.viewOverride?.opacityFactor ?? 1);
        return {
            colorMode: normalizeColorMode(this.viewOverride?.colorMode || this.colorMode),
            plainColor: this.viewOverride?.plainColor
                ? normalizePlainColor(this.viewOverride.plainColor)
                : this.plainColor,
            opacity: Math.max(0.05, Math.min(1, this.opacity * opacityFactor))
        };
    }

    applyMaterialStyle() {
        if(!this.potreeCore) return;
        const effectiveStyle = this.getEffectiveStyle();
        this.pointClouds.forEach((pointCloud) => {
            configureMaterial(pointCloud.material, this.potreeCore, {
                colorMode: effectiveStyle.colorMode,
                classificationPalette: this.classificationPalette,
                plainColor: effectiveStyle.plainColor,
                opacity: effectiveStyle.opacity,
                pointSizeScale: this.pointSizeScale
            });
        });
    }

    getSelectionPreview() {
        const effectiveStyle = this.getEffectiveStyle();
        return {
            renderer: this.type,
            status: 'lod-primary-with-business-overlay',
            visiblePoints: this.pointClouds.reduce((sum, pointCloud) => sum + (pointCloud.numVisiblePoints || 0), 0),
            colorMode: effectiveStyle.colorMode,
            canonicalColorMode: this.colorMode,
            plainColor: effectiveStyle.plainColor,
            opacity: effectiveStyle.opacity,
            overlayRole: this.overlayRole,
            bounds: this.object ? boxToArrays(this.object.getBoundingBoxWorld?.()) : null
        };
    }
}

function splitManifestUrl(manifestUrl) {
    const index = manifestUrl.lastIndexOf('/');
    if(index < 0) return { manifestName: manifestUrl, baseUrl: '' };
    return {
        manifestName: manifestUrl.slice(index + 1),
        baseUrl: manifestUrl.slice(0, index + 1)
    };
}

function configureMaterial(material, potreeCore, options = {}) {
    if(!material) return;
    const { PointColorType, PointShape, PointSizeType } = potreeCore;
    const colorMode = normalizeColorMode(options.colorMode);
    material.pointColorType = colorMode === 'classification'
        ? PointColorType.CLASSIFICATION
        : colorMode === 'plain'
            ? PointColorType.COLOR
            : PointColorType.RGB;
    if(colorMode === 'classification') {
        material.classification = normalizeClassificationPalette(options.classificationPalette);
    }
    if(colorMode === 'plain') {
        const color = normalizePlainColor(options.plainColor);
        if(material.color?.setRGB) {
            material.color.setRGB(color[0], color[1], color[2]);
        } else {
            material.color = new Color(color[0], color[1], color[2]);
        }
        if(material.uniforms?.diffuse?.value) {
            material.uniforms.diffuse.value = [color[0], color[1], color[2]];
        }
        if(material.uniforms?.uColor?.value?.setRGB) {
            material.uniforms.uColor.value.setRGB(color[0], color[1], color[2]);
        }
    }
    material.pointSizeType = PointSizeType.ADAPTIVE;
    material.shape = PointShape.CIRCLE;
    material.size = Math.max(1, 1.4 * Number(options.pointSizeScale || 1));
    material.minSize = 1;
    material.maxSize = 8;
    material.opacity = Math.max(0.05, Math.min(1, Number(options.opacity ?? 1)));
    material.transparent = material.opacity < 1;
    material.needsUpdate = true;
}

function normalizeColorMode(mode) {
    if(mode === 'classification' || mode === 'plain' || mode === 'rgb') return mode;
    return 'rgb';
}

function setLayerRecursive(object, layer) {
    object?.traverse?.((child) => child.layers.set(layer));
}

function defaultPlainColor(theme = 'light') {
    return theme === 'dark'
        ? [0.78, 0.84, 0.92]
        : [0.42, 0.46, 0.52];
}

function normalizePlainColor(value, fallback = defaultPlainColor()) {
    if(Array.isArray(value)) {
        return [
            Math.max(0, Math.min(1, Number(value[0] ?? fallback[0]))),
            Math.max(0, Math.min(1, Number(value[1] ?? fallback[1]))),
            Math.max(0, Math.min(1, Number(value[2] ?? fallback[2])))
        ];
    }
    if(value instanceof Color) return [value.r, value.g, value.b];
    if(value && typeof value === 'object') {
        return [
            Math.max(0, Math.min(1, Number(value.r ?? value.x ?? fallback[0]))),
            Math.max(0, Math.min(1, Number(value.g ?? value.y ?? fallback[1]))),
            Math.max(0, Math.min(1, Number(value.b ?? value.z ?? fallback[2])))
        ];
    }
    if(typeof value === 'string') {
        const hex = value.trim().replace('#', '');
        if(/^[0-9a-fA-F]{6}$/.test(hex)) {
            return [
                parseInt(hex.slice(0, 2), 16) / 255,
                parseInt(hex.slice(2, 4), 16) / 255,
                parseInt(hex.slice(4, 6), 16) / 255
            ];
        }
    }
    return [...fallback];
}

function defaultClassificationPalette() {
    return Object.fromEntries(
        Object.entries(LOD_CLASSIFICATION_PALETTE).map(([key, color]) => [key, vector4(color)])
    );
}

function normalizeClassificationPalette(palette = {}) {
    const normalized = defaultClassificationPalette();
    Object.entries(palette || {}).forEach(([key, value]) => {
        normalized[key] = vector4(value, normalized[key] || normalized.DEFAULT);
    });
    return normalized;
}

function vector4(value, fallback = new Vector4(0.58, 0.64, 0.70, 0.62)) {
    if(value instanceof Vector4) return value;
    if(Array.isArray(value)) {
        return new Vector4(
            Number(value[0] ?? fallback.x),
            Number(value[1] ?? fallback.y),
            Number(value[2] ?? fallback.z),
            Number(value[3] ?? fallback.w)
        );
    }
    if(value && typeof value === 'object') {
        return new Vector4(
            Number(value.x ?? value.r ?? fallback.x),
            Number(value.y ?? value.g ?? fallback.y),
            Number(value.z ?? value.b ?? fallback.z),
            Number(value.w ?? value.a ?? fallback.w)
        );
    }
    return fallback.clone();
}

function boxToArrays(box) {
    if(!(box instanceof Box3) || box.isEmpty()) return null;
    const min = new Vector3();
    const max = new Vector3();
    min.copy(box.min);
    max.copy(box.max);
    return {
        min: min.toArray(),
        max: max.toArray()
    };
}
