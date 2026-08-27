import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { gsap } from 'gsap';
import { state, alignCoordinates } from './state.js';
import { getTheme } from './theme.js';
import { ThreePointsRenderer } from './renderers/threePointsRenderer.js';
import { PotreeLodRenderer } from './renderers/potreeLodRenderer.js';
import { PointCloudLayerController } from './renderers/pointCloudLayerController.js';
import { semanticColorOf, semanticKindOf, SEMANTIC_KIND } from './renderers/semanticStyles.js';

window.THREE = THREE;

let scene, camera, renderer, controls, raycaster, mouse, viewportContainer;
let perspectiveCamera = null;
let weightOrthographicCamera = null;
let animationId;
let pointSizeScale = 1;
let globalOpacity = 1;
let weightViewTool = 'box3d';
let sceneTheme = getTheme();
let gridHelper = null;
let lastWeightColorState = { groups: [], temporaryOperations: [], selectedGroupId: null };
let weightEditableDisplay = { visible: false, mode: 'classified' };
let potreeSceneRenderer = null;
let potreeSceneRendererPromise = null;
const SCENE_LAYERS = Object.freeze({
    WORLD: 0,
    BASE_POINT: 1,
    BUSINESS_OVERLAY: 2
});
const CONTEXT_BASE_STYLE = Object.freeze({
    colorMode: 'plain',
    plainColor: [0.48, 0.52, 0.58],
    opacityFactor: 0.30
});
let visualizationContext = {
    weightEditing: false
};
let potreeRenderOptions = {
    edl: {
        enabled: false,
        pointCloudLayer: 1,
        strength: 0.35,
        radius: 1.25,
        opacity: 1,
        neighbourCount: 8
    }
};

// Group management for multiple files
// Key: 'filename' -> THREE.Group
const activeObjects = {
    pointcloud: {},
    route: {},
    voxel: {}, // Voxel is usually singular, but logic can adapt
    safety: {},
    weight: {}
};
const pointCloudControllers = new Map();

const scenePalettes = {
    light: {
        background: 0xf7f9fc,
        fog: 0xf7f9fc,
        gridMajor: 0x56aaa6,
        gridMinor: 0xd9e1e8,
        weightNeutral: [0.30, 0.33, 0.38],
        weightBase: [0.035, 0.105, 0.255],
        weightTower: [0.92, 0.12, 0.12],
        weightInsulator: [0.10, 0.34, 0.92],
        weightWire: [0.04, 0.50, 0.18],
        weightGroundWire: [0.40, 1.00, 0.60],
        selected: [0.10, 0.92, 0.78]
    },
    dark: {
        background: 0x081321,
        fog: 0x081321,
        gridMajor: 0x1db8ad,
        gridMinor: 0x22364f,
        weightNeutral: [0.58, 0.66, 0.76],
        weightBase: [0.36, 0.46, 0.60],
        weightTower: [1.00, 0.28, 0.24],
        weightInsulator: [0.32, 0.58, 1.00],
        weightWire: [0.16, 0.78, 0.36],
        weightGroundWire: [0.54, 1.00, 0.68],
        selected: [0.18, 1.0, 0.82]
    }
};

function currentPalette() {
    return scenePalettes[sceneTheme] || scenePalettes.light;
}

function applySceneTheme() {
    if(!scene) return;
    const palette = currentPalette();
    scene.background = new THREE.Color(palette.background);
    scene.fog = new THREE.FogExp2(palette.fog, sceneTheme === 'dark' ? 0.0012 : 0.00145);
    if(gridHelper) {
        const materials = Array.isArray(gridHelper.material) ? gridHelper.material : [gridHelper.material];
        if(materials[0]?.color) materials[0].color.setHex(palette.gridMajor);
        if(materials[1]?.color) materials[1].color.setHex(palette.gridMinor);
        materials.forEach((material) => {
            material.opacity = sceneTheme === 'dark' ? 0.52 : 0.62;
            material.needsUpdate = true;
        });
    }
}

function normalizeCategoryKey(cat) {
    const value = `${cat || ''}`.trim().toLowerCase();
    if (value === 'point_cloud') return 'pointcloud';
    if (value === 'manual_route' || value === 'algorithm_route' || value === 'waypoint') return 'route';
    if (value === 'safety' || value === 'violation') return 'safety';
    return value;
}

function setLayerRecursive(object, layer) {
    object?.traverse?.((child) => child.layers.set(layer));
}

function disposeObject3D(object) {
    if(!object) return;
    if(scene && object.parent) object.parent.remove(object);
    object.traverse?.((child) => {
        child.geometry?.dispose?.();
        if(child.material) {
            const materials = Array.isArray(child.material) ? child.material : [child.material];
            materials.forEach((material) => material.dispose?.());
        }
    });
}

function getOrCreatePointCloudController(id) {
    let controller = pointCloudControllers.get(id);
    if(controller) return controller;
    controller = new PointCloudLayerController({
        id,
        onRendererChanged: (nextRenderer) => {
            if(nextRenderer?.object) activeObjects.pointcloud[id] = nextRenderer.object;
            else delete activeObjects.pointcloud[id];
        }
    });
    pointCloudControllers.set(id, controller);
    return controller;
}

function hasVisibleVoxelOverlay() {
    return Object.values(activeObjects.voxel || {}).some((object) => object?.visible !== false);
}

function refreshPointCloudViewOverrides() {
    const needsContextStyle = visualizationContext.weightEditing || hasVisibleVoxelOverlay();
    pointCloudControllers.forEach((controller) => {
        controller.setViewOverride(needsContextStyle ? CONTEXT_BASE_STYLE : null);
    });
}

export function setVisualizationContext(nextContext = {}) {
    visualizationContext = {
        ...visualizationContext,
        ...nextContext
    };
    refreshPointCloudViewOverrides();
}

function getContainerSize(container) {
    const width = Math.max(1, container?.clientWidth || window.innerWidth || 1);
    const height = Math.max(1, container?.clientHeight || window.innerHeight || 1);
    return { width, height };
}

export function resizeRenderer(retry = 0) {
    const container = viewportContainer || document.getElementById('canvas-container');
    if(!container || !camera || !renderer) return;

    const width = container.clientWidth;
    const height = container.clientHeight;
    if((width < 20 || height < 20) && retry < 12) {
        requestAnimationFrame(() => resizeRenderer(retry + 1));
        return;
    }

    const safeWidth = Math.max(1, width || window.innerWidth || 1);
    const safeHeight = Math.max(1, height || window.innerHeight || 1);
    const aspect = safeWidth / safeHeight;
    if(camera.isOrthographicCamera) {
        const centerX = (camera.left + camera.right) / 2;
        const halfHeight = Math.max((camera.top - camera.bottom) / 2, 1e-6);
        camera.left = centerX - halfHeight * aspect;
        camera.right = centerX + halfHeight * aspect;
    } else {
        camera.aspect = aspect;
    }
    camera.updateProjectionMatrix();
    renderer.setSize(safeWidth, safeHeight, false);
}

export function init3D() {
    const container = document.getElementById('canvas-container');
    if(!container) return;
    if(scene && renderer) {
        resizeRenderer();
        return;
    }
    viewportContainer = container;
    const size = getContainerSize(container);

    scene = new THREE.Scene();
    applySceneTheme();

    camera = new THREE.PerspectiveCamera(60, size.width / size.height, 0.1, 20000);
    perspectiveCamera = camera;
    camera.position.set(100, 100, 100);
    camera.up.set(0, 0, 1);

    renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(size.width, size.height, false);
    renderer.domElement.className = 'scene-canvas';
    container.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    raycaster = new THREE.Raycaster();
    mouse = new THREE.Vector2();

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    ambientLight.layers.enable(SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(ambientLight);
    const dl = new THREE.DirectionalLight(0xffffff, 0.8);
    dl.position.set(100, 200, 100);
    dl.layers.enable(SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(dl);

    // Keep the empty workspace spatial without overpowering loaded inspection data.
    const palette = currentPalette();
    gridHelper = new THREE.GridHelper(500, 50, palette.gridMajor, palette.gridMinor);
    gridHelper.name = 'workspace-grid';
    gridHelper.rotateX(Math.PI / 2);
    const gridMaterials = Array.isArray(gridHelper.material) ? gridHelper.material : [gridHelper.material];
    gridMaterials.forEach((material) => {
        material.transparent = true;
        material.opacity = sceneTheme === 'dark' ? 0.52 : 0.62;
    });
    scene.add(gridHelper);
    applySceneTheme();

    // Interaction
    renderer.domElement.addEventListener('pointerdown', onPointerDown);
    window.addEventListener('resize', () => resizeRenderer());
    requestAnimationFrame(() => resizeRenderer());

    animate();
}

function animate() {
    animationId = requestAnimationFrame(animate);
    controls.update();
    updatePointCloudRenderers();
    renderSceneFrame();
}

function updatePointCloudRenderers() {
    if(!camera || !renderer) return;
    pointCloudControllers.forEach((controller) => {
        controller.update(camera, renderer);
    });
}

function getPotreePointClouds() {
    const pointClouds = [];
    pointCloudControllers.forEach((controller) => {
        const rendererAdapter = controller.renderer;
        if(rendererAdapter?.type !== 'potree-lod' || rendererAdapter.visible === false) return;
        (rendererAdapter.pointClouds || []).forEach((pointCloud) => {
            if(pointCloud && pointCloud.visible !== false) pointClouds.push(pointCloud);
        });
    });
    return pointClouds;
}

function renderSceneFrame() {
    const potreePointClouds = getPotreePointClouds();
    const previousMask = camera.layers.mask;
    const previousAutoClear = renderer.autoClear;
    const previousBackground = scene.background;

    try {
        camera.layers.mask = (1 << SCENE_LAYERS.WORLD) | (1 << SCENE_LAYERS.BASE_POINT);
        renderer.autoClear = true;
        if(potreeSceneRenderer && potreePointClouds.length) {
            potreeSceneRenderer.render({
                renderer,
                scene,
                camera,
                pointClouds: potreePointClouds
            });
        } else {
            renderer.render(scene, camera);
        }

        renderer.autoClear = false;
        renderer.clearDepth();
        scene.background = null;
        camera.layers.set(SCENE_LAYERS.BUSINESS_OVERLAY);
        renderer.render(scene, camera);
    } finally {
        scene.background = previousBackground;
        camera.layers.mask = previousMask;
        renderer.autoClear = previousAutoClear;
    }
}

function ensurePotreeSceneRenderer() {
    if(potreeSceneRenderer) return Promise.resolve(potreeSceneRenderer);
    if(potreeSceneRendererPromise) return potreeSceneRendererPromise;
    potreeSceneRendererPromise = import('potree-core')
        .then(({ PotreeRenderer }) => {
            potreeSceneRenderer = new PotreeRenderer(potreeRenderOptions);
            return potreeSceneRenderer;
        })
        .catch((error) => {
            console.warn('PotreeRenderer unavailable; falling back to Three renderer.', error);
            potreeSceneRendererPromise = null;
            return null;
        });
    return potreeSceneRendererPromise;
}

export function setPotreeRenderOptions(options = {}) {
    potreeRenderOptions = {
        ...potreeRenderOptions,
        ...options,
        edl: {
            ...(potreeRenderOptions.edl || {}),
            ...(options.edl || {})
        }
    };
    if(potreeSceneRenderer?.setEDL && potreeRenderOptions.edl) {
        potreeSceneRenderer.setEDL(potreeRenderOptions.edl);
    }
}

export function setTheme(theme = 'light') {
    sceneTheme = theme === 'dark' ? 'dark' : 'light';
    applySceneTheme();
    pointCloudControllers.forEach((controller) => controller.setTheme(sceneTheme));
    setWeightEditableDisplay({});
    updateWeightColors(
        lastWeightColorState.groups,
        lastWeightColorState.temporaryOperations,
        lastWeightColorState.selectedGroupId
    );
}

// --- Interaction ---
function onPointerDown(event) {
    // Calculate mouse position
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);

    // Check intersections with all route waypoints
    const interactables = [];
    Object.values(activeObjects.route).forEach(group => {
        if(group.visible) interactables.push(...group.children.filter(c => c.isMesh && c.name === 'waypoint'));
    });

    const intersects = raycaster.intersectObjects(interactables);
    if (intersects.length > 0) {
        const obj = intersects[0].object;
        if (obj.userData && obj.userData.id) {
            // Dispatch custom event for UI to handle
            window.dispatchEvent(new CustomEvent('waypoint-click', { detail: obj.userData }));
        }
    }
}

export function flyTo(pos) {
    if (!pos || !camera || !controls) return;
    const rawPos = Array.isArray(pos) ? pos : (pos.pos_utm || pos.position || pos.pos);
    if (!rawPos) return;

    const aligned = alignCoordinates(rawPos, null);
    const focusPoint = new THREE.Vector3(...aligned);
    const yaw = Array.isArray(pos) ? null : Number(pos.yaw ?? pos.heading);
    const pitch = Array.isArray(pos) ? null : Number(pos.pitch);

    let targetPoint = focusPoint.clone();
    let cameraPoint = focusPoint.clone().add(new THREE.Vector3(18, -18, 18));
    if (Number.isFinite(yaw)) {
        const yawRad = (90 - yaw) * Math.PI / 180;
        const pitchRad = Number.isFinite(pitch) ? pitch * Math.PI / 180 : 0;
        const direction = new THREE.Vector3(
            Math.cos(pitchRad) * Math.cos(yawRad),
            Math.cos(pitchRad) * Math.sin(yawRad),
            Math.sin(pitchRad)
        ).normalize();
        targetPoint = focusPoint.clone().add(direction.clone().multiplyScalar(18));
        cameraPoint = focusPoint.clone().add(direction.clone().multiplyScalar(-12)).add(new THREE.Vector3(0, 0, 2.5));
    }

    gsap.to(controls.target, { x: targetPoint.x, y: targetPoint.y, z: targetPoint.z, duration: 1 });
    gsap.to(camera.position, {
        x: cameraPoint.x,
        y: cameraPoint.y,
        z: cameraPoint.z,
        duration: 1.2
    });
}

// --- Rendering Logic ---

function createNumberSprite(number) {
    const canvas = document.createElement('canvas');
    canvas.width = 64;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    // Background circle
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.beginPath();
    ctx.arc(32, 32, 30, 0, Math.PI * 2);
    ctx.fill();

    // Text
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 36px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(number, 32, 32);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture, depthTest: false });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(4, 4, 1);
    sprite.renderOrder = 999; // Always on top
    return sprite;
}

function renderPointCloudGroup(data, id) {
    const group = new THREE.Group();
    const labels = data.labels || [];
    const buckets = {
        tower: { vertices: [], colors: [], size: 0.34, label: 16 },
        insulator: { vertices: [], colors: [], size: 0.64, label: 22 },
        wire: { vertices: [], colors: [], size: 0.38, label: 3 },
        other: { vertices: [], colors: [], size: 0.16, label: null }
    };

    data.points.forEach((p, i) => {
        const label = labels[i];
        let bucket = buckets.other;
        if(label === 16) bucket = buckets.tower;
        else if(label === 22) bucket = buckets.insulator;
        else if(label === 0 || label === 3) bucket = buckets.wire;

        bucket.vertices.push(...alignCoordinates(p, data.center));
        const semanticColor = data.weight_profile
            ? (data.colors[i] || currentPalette().weightBase)
            : label === 16
            ? [1.0, 0.05, 0.05]
            : label === 22
                ? [0.12, 0.28, 1.0]
                : label === 0
                    ? currentPalette().weightWire
                    : label === 3
                    ? currentPalette().weightGroundWire
                    : (data.colors[i] || [0.6, 0.7, 0.8]);
        bucket.colors.push(...semanticColor);
    });

    Object.entries(buckets).forEach(([name, bucket]) => {
        if(!bucket.vertices.length) return;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(bucket.vertices, 3));
        geo.setAttribute('color', new THREE.Float32BufferAttribute(bucket.colors, 3));
        const mat = new THREE.PointsMaterial({
            size: bucket.size * pointSizeScale,
            vertexColors: true,
            transparent: true,
            opacity: (name === 'other' ? 0.55 : 0.96) * globalOpacity,
            sizeAttenuation: true,
            depthWrite: name !== 'insulator'
        });
        const cloud = new THREE.Points(geo, mat);
        cloud.userData.baseSize = bucket.size;
        cloud.userData.baseOpacity = name === 'other' ? 0.55 : 0.96;
        cloud.name = `pointcloud-${name}`;
        group.add(cloud);
    });

    setLayerRecursive(group, SCENE_LAYERS.BASE_POINT);
    scene.add(group);
    fitCamera(group);
    return group;
}

export async function renderPointCloud(data, id) {
    const controller = getOrCreatePointCloudController(id);
    controller.invalidateLodRequests();
    const rendererAdapter = new ThreePointsRenderer({
        id,
        visible: false,
        theme: sceneTheme,
        ops: {
            render: renderPointCloudGroup,
            disposeObject: disposeObject3D,
            fitObject: fitCamera
        }
    });
    const result = await rendererAdapter.load(data, id);
    controller.commitRenderer(rendererAdapter);
    controller.setOpacity(globalOpacity);
    controller.setPointSize(pointSizeScale);
    refreshPointCloudViewOverrides();
    return result;
}

export function beginPointCloudLodRequest(id, descriptor = {}) {
    if(!id) return null;
    return getOrCreatePointCloudController(id).beginLodRequest(descriptor);
}

export function isPointCloudLodRequestCurrent(id, request) {
    return !!id && !!request && !!pointCloudControllers.get(id)?.isCurrentRequest(request);
}

export async function renderPointCloudLod(payload = {}, id, request = null) {
    if(!scene || !renderer || !id || !payload?.manifest_url) return null;
    const controller = getOrCreatePointCloudController(id);
    if(request && !controller.isCurrentRequest(request)) {
        return { renderer: 'potree-lod', status: 'ignored_stale_request' };
    }
    const existingRenderer = controller.renderer;
    if(existingRenderer?.type === 'potree-lod' && existingRenderer.manifestUrl === payload.manifest_url) {
        existingRenderer.setRenderHints?.(payload.render_hints || payload.renderHints || {});
        existingRenderer.setVisible(controller.desiredVisible);
        return controller.getInfo();
    }
    const rendererAdapter = new PotreeLodRenderer({
        id,
        scene,
        visible: false,
        baseLayer: SCENE_LAYERS.BASE_POINT,
        globalOffset: state.globalOffset || payload.global_offset || [0, 0, 0],
        fitObject: fitCamera,
        pointBudget: payload.point_budget || payload.render_hints?.point_budget || 1_800_000,
        theme: sceneTheme,
        colorMode: payload.render_hints?.color_mode,
        classificationPalette: payload.render_hints?.classification_palette,
        overlayRole: payload.render_hints?.overlay_role
    });
    try {
        const result = await rendererAdapter.load(payload);
        rendererAdapter.setOpacity(globalOpacity);
        rendererAdapter.setPointSize(pointSizeScale);
        const committed = controller.commitRenderer(rendererAdapter, request);
        if(!committed) return { ...result, status: 'ignored_stale_request' };
        refreshPointCloudViewOverrides();
        void ensurePotreeSceneRenderer();
        return result;
    } catch(error) {
        rendererAdapter.dispose();
        throw error;
    }
}

export function setPointCloudRenderHints(id, hints = {}) {
    if(!id) return null;
    const controller = pointCloudControllers.get(id);
    controller?.setRenderHints(hints);
    return controller?.getInfo() || null;
}

export function getPointCloudRendererInfo(id) {
    const info = pointCloudControllers.get(id)?.getInfo() || null;
    if(info) {
        info.sceneRenderer = potreeSceneRenderer ? 'potree-renderer' : 'three-renderer-fallback';
        info.edlEnabled = !!potreeRenderOptions.edl?.enabled;
    }
    return info;
}

function hexToRgbArray(value, fallback = [0.12, 0.72, 0.64]) {
    const hex = `${value || ''}`.replace('#', '');
    if(!/^[0-9a-fA-F]{6}$/.test(hex)) return fallback;
    return [
        parseInt(hex.slice(0, 2), 16) / 255,
        parseInt(hex.slice(2, 4), 16) / 255,
        parseInt(hex.slice(4, 6), 16) / 255
    ];
}

function weightLabelColor(label, palette = currentPalette()) {
    const numeric = Number(label);
    if(weightEditableDisplay.mode !== 'classified') return palette.weightNeutral;
    if(numeric === 16) return palette.weightTower;
    if(numeric === 22) return palette.weightInsulator;
    if(numeric === 0) return palette.weightWire;
    if(numeric === 3) return palette.weightGroundWire;
    return palette.weightNeutral;
}

function weightGeometryMask(rawPoints, operation) {
    const geometry = operation?.geometry || operation || {};
    const tool = geometry.tool || geometry.selection_tool || 'box3d';
    if(tool === 'front_xz') {
        const bounds = geometry.bounds || geometry;
        const minX = Math.min(Number(bounds.min_x ?? bounds.x1), Number(bounds.max_x ?? bounds.x2));
        const maxX = Math.max(Number(bounds.min_x ?? bounds.x1), Number(bounds.max_x ?? bounds.x2));
        const minZ = Math.min(Number(bounds.min_z ?? bounds.z1), Number(bounds.max_z ?? bounds.z2));
        const maxZ = Math.max(Number(bounds.min_z ?? bounds.z1), Number(bounds.max_z ?? bounds.z2));
        return rawPoints.map((point) => (
            Number.isFinite(minX) && Number.isFinite(maxX) && Number.isFinite(minZ) && Number.isFinite(maxZ)
            && point[0] >= minX && point[0] <= maxX && point[2] >= minZ && point[2] <= maxZ
        ));
    }
    const rect = geometry.ndc_rect || geometry.rect;
    const matrixValues = geometry.view_projection_matrix || geometry.matrix;
    if(!rect || !Array.isArray(matrixValues) || matrixValues.length !== 16) {
        return rawPoints.map(() => false);
    }
    const minX = Math.min(Number(rect.min_x ?? rect.x1), Number(rect.max_x ?? rect.x2));
    const maxX = Math.max(Number(rect.min_x ?? rect.x1), Number(rect.max_x ?? rect.x2));
    const minY = Math.min(Number(rect.min_y ?? rect.y1), Number(rect.max_y ?? rect.y2));
    const maxY = Math.max(Number(rect.min_y ?? rect.y1), Number(rect.max_y ?? rect.y2));
    const matrix = new THREE.Matrix4().fromArray(matrixValues);
    return rawPoints.map((point) => {
        const clip = new THREE.Vector4(point[0], point[1], point[2], 1).applyMatrix4(matrix);
        if(Math.abs(clip.w) < 1e-9) return false;
        const x = clip.x / clip.w;
        const y = clip.y / clip.w;
        const z = clip.z / clip.w;
        return x >= minX && x <= maxX && y >= minY && y <= maxY && z >= -1 && z <= 1;
    });
}

function evaluateWeightOperations(rawPoints, operations = []) {
    let current = rawPoints.map(() => false);
    operations.forEach((operation) => {
        const mode = operation.mode || operation.selection_mode || 'new';
        if(mode === 'invert') {
            current = current.map((value) => !value);
            return;
        }
        const selected = weightGeometryMask(rawPoints, operation);
        if(mode === 'add') current = current.map((value, index) => value || selected[index]);
        else if(mode === 'subtract') current = current.map((value, index) => value && !selected[index]);
        else current = selected;
    });
    return current;
}

function summarizeSelectionDataset(points = [], labels = [], operations = []) {
    const summary = {
        sampleCount: Array.isArray(points) ? points.length : 0,
        selectedSampleCount: 0,
        towerCount: 0,
        insulatorCount: 0,
        conductorCount: 0,
        groundWireCount: 0,
        otherCount: 0
    };
    if(!summary.sampleCount || !operations?.length) return summary;
    const mask = evaluateWeightOperations(points, operations);
    mask.forEach((selected, index) => {
        if(!selected) return;
        summary.selectedSampleCount += 1;
        const label = Number(labels[index]);
        if(label === 16) summary.towerCount += 1;
        else if(label === 22) summary.insulatorCount += 1;
        else if(label === 0) summary.conductorCount += 1;
        else if(label === 3) summary.groundWireCount += 1;
        else summary.otherCount += 1;
    });
    return summary;
}

export function summarizeWeightSelection(operations = [], data = null) {
    const editor = activeObjects.weight['weight-editor'];
    const targetPoints = data?.points || editor?.userData.rawPoints || [];
    const targetLabels = data?.labels || editor?.userData.labels || [];
    const contextPoints = data?.context_points || [];
    const contextLabels = data?.context_labels || [];
    const target = summarizeSelectionDataset(targetPoints, targetLabels, operations);
    const context = summarizeSelectionDataset(contextPoints, contextLabels, operations);
    const targetSelected = target.towerCount + target.insulatorCount;
    const contextSelected = context.conductorCount + context.groundWireCount;
    return {
        target,
        context,
        selectedSampleCount: target.selectedSampleCount + context.selectedSampleCount,
        targetSelectedSampleCount: targetSelected,
        safetySelectedSampleCount: contextSelected,
        towerCount: target.towerCount,
        insulatorCount: target.insulatorCount,
        conductorCount: context.conductorCount,
        groundWireCount: context.groundWireCount,
        otherCount: target.otherCount + context.otherCount
    };
}

function renderWeightSelectionGuides(rawPoints = [], cloud = null, groups = [], temporaryOperations = [], selectedGroupId = null) {
    removeObject('weight', 'weight-selection-guides');
    if(!scene || !cloud?.geometry || !rawPoints.length) return;
    const positions = cloud.geometry.getAttribute('position');
    if(!positions) return;
    const guideGroup = new THREE.Group();
    guideGroup.name = 'weight-selection-guides';

    const addGuide = (mask, color, options = {}) => {
        const box = new THREE.Box3();
        let count = 0;
        mask.forEach((selected, index) => {
            if(!selected || index >= positions.count) return;
            box.expandByPoint(new THREE.Vector3(
                positions.getX(index),
                positions.getY(index),
                positions.getZ(index)
            ));
            count += 1;
        });
        if(!count || box.isEmpty()) return;
        const helper = new THREE.Box3Helper(box, color);
        helper.name = options.name || 'weight-selection-guide';
        helper.renderOrder = 12;
        helper.material.transparent = true;
        helper.material.opacity = options.opacity || 0.78;
        helper.material.depthTest = false;
        helper.material.depthWrite = false;
        helper.userData = {
            pointCount: count,
            groupId: options.groupId || null,
            temporary: !!options.temporary
        };
        guideGroup.add(helper);
    };

    groups.forEach((group, index) => {
        if(group.enabled === false || group.visible === false) return;
        const operations = group.selection_geometry?.operations || [];
        if(!operations.length) return;
        const mask = evaluateWeightOperations(rawPoints, operations);
        const selected = group.group_id === selectedGroupId;
        addGuide(mask, group.color || '#14b8a6', {
            groupId: group.group_id,
            name: selected ? 'weight-selection-guide-selected' : 'weight-selection-guide',
            opacity: selected ? 0.96 : 0.62 + Math.min(index, 3) * 0.04
        });
    });

    if(temporaryOperations.length) {
        const mask = evaluateWeightOperations(rawPoints, temporaryOperations);
        addGuide(mask, sceneTheme === 'dark' ? '#5eead4' : '#0f766e', {
            temporary: true,
            name: 'weight-selection-guide-temporary',
            opacity: 0.95
        });
    }

    if(!guideGroup.children.length) return;
    setLayerRecursive(guideGroup, SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(guideGroup);
    activeObjects.weight['weight-selection-guides'] = guideGroup;
}

function getWeightEditorParts() {
    const group = activeObjects.weight['weight-editor'];
    return {
        group,
        sample: group?.getObjectByName?.('weight-editor-sample') || null,
        highlight: group?.getObjectByName?.('weight-editor-highlight') || null
    };
}

export function renderWeightEditable(data, id = 'weight-editor') {
    removeObject('weight', id);
    const group = new THREE.Group();
    const vertices = [];
    const colors = [];
    const palette = currentPalette();
    const labels = data.labels || [];
    (data.points || []).forEach((point, index) => {
        vertices.push(...alignCoordinates(point, data.center));
        colors.push(...weightLabelColor(labels[index], palette));
    });
    const sampleGeometry = new THREE.BufferGeometry();
    sampleGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    sampleGeometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    const sampleMaterial = new THREE.PointsMaterial({
        size: 0.22 * pointSizeScale,
        vertexColors: true,
        transparent: true,
        opacity: 0.68,
        sizeAttenuation: true,
        depthWrite: false
    });
    const sample = new THREE.Points(sampleGeometry, sampleMaterial);
    sample.name = 'weight-editor-sample';
    sample.visible = !!weightEditableDisplay.visible;
    sample.userData.baseSize = 0.22;
    sample.userData.baseOpacity = 0.68;

    const highlightGeometry = new THREE.BufferGeometry();
    highlightGeometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    highlightGeometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    const highlightMaterial = new THREE.PointsMaterial({
        size: 0.54 * pointSizeScale,
        vertexColors: true,
        transparent: true,
        opacity: 1,
        sizeAttenuation: true,
        depthWrite: false
    });
    const highlight = new THREE.Points(highlightGeometry, highlightMaterial);
    highlight.name = 'weight-editor-highlight';
    highlight.renderOrder = 10;
    highlight.userData.baseSize = 0.54;
    highlight.userData.baseOpacity = 1;

    group.name = id;
    group.userData.rawPoints = data.points || [];
    group.userData.labels = labels;
    group.add(sample);
    group.add(highlight);
    setLayerRecursive(group, SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(group);
    activeObjects.weight[id] = group;
    fitCamera(group);
}

export function setWeightEditableDisplay(options = {}) {
    if(Object.prototype.hasOwnProperty.call(options, 'visible')) {
        weightEditableDisplay.visible = !!options.visible;
    }
    if(['classified', 'plain'].includes(options.mode)) {
        weightEditableDisplay.mode = options.mode;
    }
    const { group, sample } = getWeightEditorParts();
    if(sample) {
        sample.visible = !!weightEditableDisplay.visible;
        const labels = group?.userData.labels || [];
        const colorAttribute = sample.geometry?.getAttribute('color');
        if(colorAttribute) {
            const palette = currentPalette();
            for(let index = 0; index < colorAttribute.count; index += 1) {
                const color = weightLabelColor(labels[index], palette);
                colorAttribute.setXYZ(index, color[0], color[1], color[2]);
            }
            colorAttribute.needsUpdate = true;
        }
        if(sample.material) {
            sample.material.opacity = weightEditableDisplay.mode === 'plain' ? 0.52 : 0.68;
            sample.material.size = (sample.userData.baseSize || 0.22) * pointSizeScale;
            sample.material.needsUpdate = true;
        }
    }
}

export function updateWeightColors(groups = [], temporaryOperations = [], selectedGroupId = null) {
    lastWeightColorState = {
        groups,
        temporaryOperations,
        selectedGroupId
    };
    const { group, sample, highlight } = getWeightEditorParts();
    if(!group || !sample || !highlight) return;
    const palette = currentPalette();
    const rawPoints = group.userData.rawPoints || [];
    const labels = group.userData.labels || [];
    const assignments = rawPoints.map(() => null);
    const winningPriorities = rawPoints.map(() => -1);
    const winningRevisions = rawPoints.map(() => -1);
    const priority = { normal: 1, needed: 2, important: 3 };
    groups.forEach((item, groupIndex) => {
        if(item.enabled === false) return;
        const operations = item.selection_geometry?.operations || [];
        const mask = evaluateWeightOperations(rawPoints, operations);
        const semanticFilter = new Set(item.semantic_filter || ['tower', 'insulator']);
        const itemPriority = priority[item.level] || 1;
        const itemRevision = Number(item.revision_seq || groupIndex + 1);
        mask.forEach((selected, pointIndex) => {
            const semanticAllowed = labels[pointIndex] === 16
                ? semanticFilter.has('tower')
                : labels[pointIndex] === 22 && semanticFilter.has('insulator');
            const wins = itemPriority > winningPriorities[pointIndex]
                || (
                    itemPriority === winningPriorities[pointIndex]
                    && itemRevision >= winningRevisions[pointIndex]
                );
            if(selected && semanticAllowed && wins) {
                assignments[pointIndex] = item;
                winningPriorities[pointIndex] = itemPriority;
                winningRevisions[pointIndex] = itemRevision;
            }
        });
    });
    const temporaryMask = evaluateWeightOperations(rawPoints, temporaryOperations);
    const sourcePositions = sample.geometry.getAttribute('position');
    const highlightPositions = [];
    const highlightColors = [];
    assignments.forEach((item, pointIndex) => {
        const temporary = !!temporaryMask[pointIndex];
        if(!temporary && (!item || item.visible === false)) return;
        let color = item ? hexToRgbArray(item.color) : weightLabelColor(labels[pointIndex], palette);
        if(item?.visible !== false && item?.group_id === selectedGroupId) {
            color = color.map((value) => Math.min(1, value * 1.24 + 0.08));
        }
        if(temporary) color = palette.selected;
        highlightPositions.push(
            sourcePositions.getX(pointIndex),
            sourcePositions.getY(pointIndex),
            sourcePositions.getZ(pointIndex)
        );
        highlightColors.push(color[0], color[1], color[2]);
    });
    const nextGeometry = new THREE.BufferGeometry();
    nextGeometry.setAttribute('position', new THREE.Float32BufferAttribute(highlightPositions, 3));
    nextGeometry.setAttribute('color', new THREE.Float32BufferAttribute(highlightColors, 3));
    highlight.geometry.dispose();
    highlight.geometry = nextGeometry;
    highlight.userData.pointCount = highlightPositions.length / 3;
    renderWeightSelectionGuides(rawPoints, sample, groups, temporaryOperations, selectedGroupId);
}

export function focusWeightGroup(groups = [], groupId = null) {
    const { group: editor, sample } = getWeightEditorParts();
    const group = groups.find((item) => item.group_id === groupId);
    if(!editor || !sample || !group) return;
    const rawPoints = editor.userData.rawPoints || [];
    const mask = evaluateWeightOperations(rawPoints, group.selection_geometry?.operations || []);
    const positions = sample.geometry.getAttribute('position');
    const box = new THREE.Box3();
    mask.forEach((selected, index) => {
        if(selected) box.expandByPoint(new THREE.Vector3(
            positions.getX(index),
            positions.getY(index),
            positions.getZ(index)
        ));
    });
    if(!box.isEmpty()) fitBox(box);
}

export function buildWeightSelectionGeometry(pixelRect, tool = 'box3d') {
    if(!camera || !renderer) return null;
    camera.updateMatrixWorld(true);
    const width = Math.max(renderer.domElement.clientWidth, 1);
    const height = Math.max(renderer.domElement.clientHeight, 1);
    const x1 = (pixelRect.left / width) * 2 - 1;
    const x2 = (pixelRect.right / width) * 2 - 1;
    const y1 = -((pixelRect.top / height) * 2 - 1);
    const y2 = -((pixelRect.bottom / height) * 2 - 1);
    if(tool === 'front_xz') {
        const a = new THREE.Vector3(x1, y1, 0).unproject(camera);
        const b = new THREE.Vector3(x2, y2, 0).unproject(camera);
        const offset = state.globalOffset || [0, 0, 0];
        return {
            tool,
            bounds: {
                min_x: Math.min(a.x, b.x) + offset[0],
                max_x: Math.max(a.x, b.x) + offset[0],
                min_z: Math.min(a.z, b.z) + offset[2],
                max_z: Math.max(a.z, b.z) + offset[2]
            }
        };
    }
    const viewProjection = new THREE.Matrix4().multiplyMatrices(
        camera.projectionMatrix,
        camera.matrixWorldInverse
    );
    const offset = state.globalOffset || [0, 0, 0];
    const translation = new THREE.Matrix4().makeTranslation(-offset[0], -offset[1], -offset[2]);
    viewProjection.multiply(translation);
    return {
        tool,
        ndc_rect: {
            min_x: Math.min(x1, x2),
            max_x: Math.max(x1, x2),
            min_y: Math.min(y1, y2),
            max_y: Math.max(y1, y2)
        },
        view_projection_matrix: viewProjection.toArray()
    };
}

function replaceControls(nextCamera, rotateEnabled = true) {
    if(controls) controls.dispose();
    camera = nextCamera;
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.enableRotate = rotateEnabled;
    controls.enablePan = true;
    controls.enableZoom = true;
    controls.screenSpacePanning = true;
    if(!rotateEnabled) {
        controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
        controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
        controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
        controls.touches.ONE = THREE.TOUCH.PAN;
        controls.touches.TWO = THREE.TOUCH.DOLLY_PAN;
    }
}

function configureWeightPointMaterial(tool) {
    const { sample, highlight } = getWeightEditorParts();
    if(!sample?.material || !highlight?.material) return;
    const isFront = tool === 'front_xz';
    [sample, highlight].forEach((cloud) => {
        cloud.material.sizeAttenuation = !isFront;
        cloud.material.size = isFront
            ? Math.max(cloud === highlight ? 2.4 : 1.5, (cloud === highlight ? 3.1 : 2.2) * pointSizeScale)
            : (cloud.userData.baseSize || 0.46) * pointSizeScale;
        cloud.material.opacity = isFront ? 1 : (cloud.userData.baseOpacity || 0.96);
        cloud.material.needsUpdate = true;
    });
}

export function setWeightNavigationEnabled(enabled = true) {
    if(controls) controls.enabled = !!enabled;
}

export function setWeightView(tool = 'box3d') {
    const group = activeObjects.weight['weight-editor'];
    if(!group || !renderer) return;
    const box = new THREE.Box3().setFromObject(group);
    if(box.isEmpty()) return;
    weightViewTool = tool;
    configureWeightPointMaterial(tool);
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    if(tool === 'front_xz') {
        const aspect = Math.max(renderer.domElement.clientWidth, 1) / Math.max(renderer.domElement.clientHeight, 1);
        const halfHeight = Math.max(size.z * 0.65, size.x / Math.max(aspect, 0.1) * 0.65, 10);
        weightOrthographicCamera = new THREE.OrthographicCamera(
            -halfHeight * aspect,
            halfHeight * aspect,
            halfHeight,
            -halfHeight,
            0.1,
            20000
        );
        weightOrthographicCamera.position.set(center.x, center.y - Math.max(size.y * 3, 100), center.z);
        weightOrthographicCamera.up.set(0, 0, 1);
        weightOrthographicCamera.lookAt(center);
        replaceControls(weightOrthographicCamera, false);
        controls.target.copy(center);
        controls.update();
        return;
    }
    if(!perspectiveCamera) return;
    replaceControls(perspectiveCamera, true);
    fitBox(box);
}

export function toggleWorkspaceObjects(visible, options = {}) {
    const keepPointCloud = !!options.keepPointCloud;
    ['voxel', 'route', 'safety'].forEach((key) => {
        Object.values(activeObjects[key] || {}).forEach((object) => {
            object.visible = !!visible;
        });
    });
    pointCloudControllers.forEach((controller) => {
        controller.setVisible(keepPointCloud ? true : visible);
    });
    refreshPointCloudViewOverrides();
}

export function clearWeightEditable() {
    Object.keys(activeObjects.weight).forEach((id) => removeObject('weight', id));
    if(camera !== perspectiveCamera && perspectiveCamera) {
        replaceControls(perspectiveCamera, true);
    }
}

export function renderRoute(waypoints, id, type='manual') {
    removeObject('route', id);

    const group = new THREE.Group();
    const pts = [];

    waypoints.forEach(wp => {
        const posArr = alignCoordinates(wp.pos_utm, null);
        const pos = new THREE.Vector3(...posArr);
        pts.push(pos);

        // Waypoint Sphere
        const isTask = (wp.point_type ? wp.point_type === 'task' : wp.action === 'photo');
        const color = type === 'manual'
            ? (isTask ? 0xc7446f : 0x64748b)
            : (isTask ? 0x0f8f7a : 0x8aa29e);
        const sphere = new THREE.Mesh(
            new THREE.SphereGeometry(isTask ? 0.95 : 0.48),
            new THREE.MeshStandardMaterial({color, roughness: 0.3})
        );
        sphere.position.copy(pos);
        sphere.name = 'waypoint';
        sphere.userData = {
            id: wp.id,
            pos: wp.pos_utm,
            pos_utm: wp.pos_utm,
            yaw: wp.yaw,
            pitch: wp.pitch,
            fileId: id,
            routeType: type,
            waypoint: { ...wp, fileId: id, routeType: type }
        };
        group.add(sphere);

        // Number Label (Sprite)
        if(isTask) {
            const label = createNumberSprite(wp.id);
            label.position.copy(pos).add(new THREE.Vector3(0, 0, 2)); // Float above
            group.add(label);
        }

        // Direction Arrow
        if(wp.yaw !== undefined) {
             const arrow = createArrow(pos, wp.pitch, wp.yaw, isTask ? 0x22c55e : 0x94a3b8);
             group.add(arrow);
             if(isTask) group.add(createCameraFrustum(pos, wp.pitch || 0, wp.yaw || 0, type === 'manual' ? 0xc7446f : 0x0f8f7a));
        }
    });

    // Line
    const routeColor = type === 'manual' ? 0xc7446f : 0x00a6ff;
    const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({color: routeColor, linewidth: 2})
    );
    group.add(line);

    setLayerRecursive(group, SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(group);
    activeObjects.route[id] = group;

    if(type === 'best') fitCamera(group);
}

export function renderVoxels(data, center, id) {
    resizeRenderer();
    // 淇濇寔鍗曚緥鏄剧ず锛氭覆鏌撴柊鐨勪綋绱犲墠绉婚櫎宸叉湁浣撶礌鍥惧眰
    Object.keys(activeObjects.voxel).forEach((key) => removeObject('voxel', key));

    const group = new THREE.Group();
    const buckets = {
        tower: { vertices: [], colors: [], size: 0.52, opacity: 0.9 },
        insulator: { vertices: [], colors: [], size: 0.68, opacity: 0.92 },
        wire: { vertices: [], colors: [], size: 0.48, opacity: 0.9 },
        other: { vertices: [], colors: [], size: 0.34, opacity: 0.45 }
    };

    data.voxels.forEach((v) => {
        const pos = alignCoordinates(v.pos, center);
        const semanticKind = semanticKindOf(v);
        const bucket = (semanticKind === SEMANTIC_KIND.CONDUCTOR || semanticKind === SEMANTIC_KIND.GROUND_WIRE)
            ? buckets.wire
            : semanticKind === SEMANTIC_KIND.INSULATOR
            ? buckets.insulator
            : (semanticKind === SEMANTIC_KIND.TOWER ? buckets.tower : buckets.other);
        const color = semanticColorOf(v);
        bucket.vertices.push(...pos);
        bucket.colors.push(...color);
    });

    Object.entries(buckets).forEach(([name, bucket]) => {
        if(!bucket.vertices.length) return;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(bucket.vertices, 3));
        geo.setAttribute('color', new THREE.Float32BufferAttribute(bucket.colors, 3));
        const mat = new THREE.PointsMaterial({
            size: bucket.size * pointSizeScale,
            vertexColors: true,
            transparent: true,
            opacity: bucket.opacity * globalOpacity,
            sizeAttenuation: true,
            depthWrite: false
        });
        const points = new THREE.Points(geo, mat);
        points.userData.baseSize = bucket.size;
        points.userData.baseOpacity = bucket.opacity;
        points.name = `voxel-${name}`;
        group.add(points);
    });

    const box = new THREE.Box3().setFromObject(group);
    if(!box.isEmpty()) {
        const helper = new THREE.Box3Helper(box, 0xff2020);
        helper.material.transparent = true;
        helper.material.opacity = 0.7;
        group.add(helper);
    }

    setLayerRecursive(group, SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(group);
    activeObjects.voxel[id] = group;
    refreshPointCloudViewOverrides();
    fitCamera(group);
}

export function removeObject(cat, id) {
    const key = normalizeCategoryKey(cat);
    if(key === 'pointcloud') {
        const controller = pointCloudControllers.get(id);
        controller?.dispose();
        pointCloudControllers.delete(id);
        delete activeObjects.pointcloud[id];
        return;
    }
    if (scene && activeObjects[key] && activeObjects[key][id]) {
        disposeObject3D(activeObjects[key][id]);
        delete activeObjects[key][id];
        if(key === 'voxel') refreshPointCloudViewOverrides();
    }
}

export function toggleObjectVisibility(cat, id, visible) {
    const key = normalizeCategoryKey(cat);
    const controller = key === 'pointcloud' ? pointCloudControllers.get(id) : null;
    if(controller) {
        controller.setVisible(visible);
        return;
    }
    if (activeObjects[key] && activeObjects[key][id]) {
        activeObjects[key][id].visible = visible;
        if(key === 'voxel') refreshPointCloudViewOverrides();
    }
}

export function focusObject(cat, id) {
    const key = normalizeCategoryKey(cat);
    const controller = key === 'pointcloud' ? pointCloudControllers.get(id) : null;
    if(controller) {
        controller.focus();
        return;
    }
    const obj = activeObjects[key]?.[id];
    if(obj) fitCamera(obj);
}

export function resetView() {
    if(!scene || !camera || !controls) return;
    const box = new THREE.Box3();
    let hasObject = false;
    pointCloudControllers.forEach((controller) => {
        const object = controller.renderer?.object;
        if(!object || object.visible === false) return;
        const objBox = new THREE.Box3().setFromObject(object);
        if(!objBox.isEmpty()) {
            box.union(objBox);
            hasObject = true;
        }
    });
    ['voxel', 'route', 'weight'].forEach((key) => {
        Object.values(activeObjects[key] || {}).forEach((obj) => {
            if(!obj.visible) return;
            const objBox = new THREE.Box3().setFromObject(obj);
            if(!objBox.isEmpty()) {
                box.union(objBox);
                hasObject = true;
            }
        });
    });
    if(hasObject) fitBox(box);
    else {
        gsap.to(camera.position, { x: 100, y: 100, z: 100, duration: 0.8 });
        gsap.to(controls.target, { x: 0, y: 0, z: 0, duration: 0.8 });
    }
}

export function setPointSizeScale(scale = 1) {
    pointSizeScale = Math.max(0.25, Math.min(3, Number(scale) || 1));
    ['voxel', 'weight'].forEach((key) => {
        Object.values(activeObjects[key] || {}).forEach((obj) => {
            obj.traverse((child) => {
                if(child.isPoints && child.material) {
                    child.material.size = key === 'weight' && weightViewTool === 'front_xz'
                        ? Math.max(1.5, 2.2 * pointSizeScale)
                        : (child.userData.baseSize || child.material.size || 1) * pointSizeScale;
                    child.material.needsUpdate = true;
                }
            });
        });
    });
    pointCloudControllers.forEach((controller) => {
        controller.setPointSize(pointSizeScale);
    });
}

export function setGlobalOpacity(opacity = 1) {
    globalOpacity = Math.max(0.15, Math.min(1, Number(opacity) || 1));
    ['voxel'].forEach((key) => {
        Object.values(activeObjects[key] || {}).forEach((obj) => {
            obj.traverse((child) => {
                if(child.material) {
                    const materials = Array.isArray(child.material) ? child.material : [child.material];
                    materials.forEach((mat) => {
                        mat.userData = mat.userData || {};
                        if(!Number.isFinite(mat.userData.baseOpacity)) {
                            mat.userData.baseOpacity = Number.isFinite(mat.opacity) ? mat.opacity : 1;
                        }
                        const base = Number.isFinite(child.userData.baseOpacity)
                            ? child.userData.baseOpacity
                            : mat.userData.baseOpacity;
                        mat.transparent = true;
                        mat.opacity = base * globalOpacity;
                        mat.needsUpdate = true;
                    });
                }
            });
        });
    });
    pointCloudControllers.forEach((controller) => {
        controller.setOpacity(globalOpacity);
    });
}

export function renderSafetyViolations(result = {}) {
    removeObject('safety', 'latest');
    if(!scene || !result || !Array.isArray(result.violations) || !result.violations.length) return;
    const group = new THREE.Group();
    const routeGroup = activeObjects.route[result.filename] || Object.values(activeObjects.route)[0];
    if(!routeGroup) return;
    const waypointMeshes = routeGroup.children.filter((child) => child.isMesh && child.name === 'waypoint');
    const byId = new Map();
    waypointMeshes.forEach((mesh, index) => {
        byId.set(String(mesh.userData.id), mesh);
        byId.set(String(index + 1), mesh);
        byId.set(String(index), mesh);
    });

    result.violations.slice(0, 40).forEach((violation) => {
        if(violation.type === 'segment') {
            const from = byId.get(String(violation.from));
            const to = byId.get(String(violation.to));
            if(from && to) {
                const line = new THREE.Line(
                    new THREE.BufferGeometry().setFromPoints([from.position, to.position]),
                    new THREE.LineBasicMaterial({ color: 0xdc2626, linewidth: 3, transparent: true, opacity: 0.95 })
                );
                group.add(line);
            }
            return;
        }
        const mesh = byId.get(String(violation.index));
        if(!mesh) return;
        const marker = new THREE.Mesh(
            new THREE.SphereGeometry(1.55, 20, 20),
            new THREE.MeshBasicMaterial({ color: 0xdc2626, transparent: true, opacity: 0.62, depthTest: false })
        );
        marker.position.copy(mesh.position);
        marker.name = 'safety-violation';
        marker.userData = violation;
        group.add(marker);
    });
    if(!group.children.length) return;
    setLayerRecursive(group, SCENE_LAYERS.BUSINESS_OVERLAY);
    scene.add(group);
    activeObjects.safety.latest = group;
}

function createArrow(pos, pitch, yaw, color=0x00ff00) {
    const yawRad = (90 - yaw) * Math.PI / 180;
    const pitchRad = pitch * Math.PI / 180;
    const x = Math.cos(pitchRad) * Math.cos(yawRad);
    const y = Math.cos(pitchRad) * Math.sin(yawRad);
    const z = Math.sin(pitchRad);
    return new THREE.ArrowHelper(new THREE.Vector3(x, y, z).normalize(), pos, 3, color, 0.6, 0.4);
}

function createCameraFrustum(pos, pitch, yaw, color=0x0f8f7a) {
    const yawRad = (90 - yaw) * Math.PI / 180;
    const pitchRad = pitch * Math.PI / 180;
    const forward = new THREE.Vector3(
        Math.cos(pitchRad) * Math.cos(yawRad),
        Math.cos(pitchRad) * Math.sin(yawRad),
        Math.sin(pitchRad)
    ).normalize();
    const up = new THREE.Vector3(0, 0, 1);
    const right = new THREE.Vector3().crossVectors(forward, up).normalize();
    const realUp = new THREE.Vector3().crossVectors(right, forward).normalize();
    const length = 6;
    const width = 2.6;
    const height = 1.8;
    const center = pos.clone().add(forward.clone().multiplyScalar(length));
    const corners = [
        center.clone().add(right.clone().multiplyScalar(width)).add(realUp.clone().multiplyScalar(height)),
        center.clone().add(right.clone().multiplyScalar(-width)).add(realUp.clone().multiplyScalar(height)),
        center.clone().add(right.clone().multiplyScalar(-width)).add(realUp.clone().multiplyScalar(-height)),
        center.clone().add(right.clone().multiplyScalar(width)).add(realUp.clone().multiplyScalar(-height))
    ];
    const points = [];
    corners.forEach((corner) => points.push(pos, corner));
    points.push(corners[0], corners[1], corners[1], corners[2], corners[2], corners[3], corners[3], corners[0]);
    return new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.38 })
    );
}

function fitCamera(obj) {
    resizeRenderer();
    const box = new THREE.Box3().setFromObject(obj);
    if(box.isEmpty()) return;
    fitBox(box);
}

function fitBox(box) {
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const max = Math.max(size.x, size.y, size.z);
    gsap.to(camera.position, {x: center.x+max*1.5, y: center.y+max*1.5, z: center.z+max*1.5, duration: 1});
    gsap.to(controls.target, {x: center.x, y: center.y, z: center.z, duration: 1});
}

