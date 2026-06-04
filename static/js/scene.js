import { state, alignCoordinates } from './state.js?v=3.9';

let scene, camera, renderer, controls, raycaster, mouse, viewportContainer;
let animationId;
let pointSizeScale = 1;
let globalOpacity = 1;

// Group management for multiple files
// Key: 'filename' -> THREE.Group
const activeObjects = {
    pointcloud: {},
    route: {},
    voxel: {}, // Voxel is usually singular, but logic can adapt
    safety: {}
};

function normalizeCategoryKey(cat) {
    const value = `${cat || ''}`.trim().toLowerCase();
    if (value === 'point_cloud') return 'pointcloud';
    if (value === 'manual_route' || value === 'algorithm_route' || value === 'waypoint') return 'route';
    if (value === 'safety' || value === 'violation') return 'safety';
    return value;
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
    camera.aspect = safeWidth / safeHeight;
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
    scene.background = new THREE.Color(0xf3faf6);
    scene.fog = new THREE.FogExp2(0xf3faf6, 0.0016);

    camera = new THREE.PerspectiveCamera(60, size.width / size.height, 0.1, 20000);
    camera.position.set(100, 100, 100);
    camera.up.set(0, 0, 1);

    renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(size.width, size.height, false);
    renderer.domElement.className = 'scene-canvas';
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    raycaster = new THREE.Raycaster();
    mouse = new THREE.Vector2();

    // Lights
    scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dl = new THREE.DirectionalLight(0xffffff, 0.8);
    dl.position.set(100, 200, 100);
    scene.add(dl);

    // Helpers
    scene.add(new THREE.GridHelper(500, 50, 0x2aa899, 0xd1ddd8).rotateX(Math.PI/2));
    scene.add(new THREE.AxesHelper(10));

    // Interaction
    renderer.domElement.addEventListener('pointerdown', onPointerDown);
    window.addEventListener('resize', () => resizeRenderer());
    requestAnimationFrame(() => resizeRenderer());

    animate();
}

function animate() {
    animationId = requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
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

export function renderPointCloud(data, id) {
    // If exists, remove first
    removeObject('pointcloud', id);

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
        const semanticColor = label === 16
            ? [1.0, 0.05, 0.05]
            : label === 22
                ? [0.12, 0.28, 1.0]
                : (label === 0 || label === 3)
                    ? [0.0, 0.85, 1.0]
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

    scene.add(group);

    activeObjects.pointcloud[id] = group;
    fitCamera(group);
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

    scene.add(group);
    activeObjects.route[id] = group;

    if(type === 'best') fitCamera(group);
}

export function renderVoxels(data, center, id) {
    resizeRenderer();
    // 保持单例显示：渲染新的体素前移除已有体素图层
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
        const bucket = (v.category === 'wire' || v.category === 'ground_wire' || v.semantic === 'wire' || v.semantic === 'ground_wire')
            ? buckets.wire
            : (v.category === 'insulator' || v.type === 3)
            ? buckets.insulator
            : (v.category === 'tower' || v.type === 2 ? buckets.tower : buckets.other);
        const color = bucket === buckets.insulator
            ? [0.12, 0.28, 1.0]
            : bucket === buckets.wire
                ? [0.0, 0.85, 1.0]
                : bucket === buckets.tower ? [1.0, 0.05, 0.05] : [0.55, 0.62, 0.74];
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

    scene.add(group);
    activeObjects.voxel[id] = group;
    fitCamera(group);
}

export function removeObject(cat, id) {
    const key = normalizeCategoryKey(cat);
    if (scene && activeObjects[key] && activeObjects[key][id]) {
        scene.remove(activeObjects[key][id]);

        // Memory cleanup
        activeObjects[key][id].traverse(c => {
            if(c.geometry) c.geometry.dispose();
            if(c.material) {
                if(Array.isArray(c.material)) c.material.forEach(m => m.dispose());
                else c.material.dispose();
            }
        });

        delete activeObjects[key][id];
    }
}

export function toggleObjectVisibility(cat, id, visible) {
    const key = normalizeCategoryKey(cat);
    if (activeObjects[key] && activeObjects[key][id]) {
        activeObjects[key][id].visible = visible;
    }
}

export function focusObject(cat, id) {
    const key = normalizeCategoryKey(cat);
    const obj = activeObjects[key]?.[id];
    if(obj) fitCamera(obj);
}

export function resetView() {
    if(!scene || !camera || !controls) return;
    const box = new THREE.Box3();
    let hasObject = false;
    ['pointcloud', 'voxel', 'route'].forEach((key) => {
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
    ['pointcloud', 'voxel'].forEach((key) => {
        Object.values(activeObjects[key] || {}).forEach((obj) => {
            obj.traverse((child) => {
                if(child.isPoints && child.material) {
                    child.material.size = (child.userData.baseSize || child.material.size || 1) * pointSizeScale;
                    child.material.needsUpdate = true;
                }
            });
        });
    });
}

export function setGlobalOpacity(opacity = 1) {
    globalOpacity = Math.max(0.15, Math.min(1, Number(opacity) || 1));
    ['pointcloud', 'voxel', 'route'].forEach((key) => {
        Object.values(activeObjects[key] || {}).forEach((obj) => {
            obj.traverse((child) => {
                if(child.material) {
                    const materials = Array.isArray(child.material) ? child.material : [child.material];
                    materials.forEach((mat) => {
                        const base = child.userData.baseOpacity || mat.userData?.baseOpacity || mat.opacity || 1;
                        mat.transparent = true;
                        mat.opacity = base * globalOpacity;
                        mat.needsUpdate = true;
                    });
                }
            });
        });
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
