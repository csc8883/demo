import { state } from './state.js';

// 辅助函数：自动添加认证头
function getAuthHeaders() {
    const headers = {};
    // 从 state 中获取当前用户名，并放入 header
    // 关键：state.user 必须由 app.js 写入同一个 state 实例
    if (state.user && state.user.name) {
        headers['X-User-Name'] = state.user.name;
    }
    if (state.user && state.user.token) {
        headers['X-Auth-Token'] = state.user.token;
    }
    return headers;
}

async function fetchJsonSafe(url, options = {}) {
    // 1. 自动合并 Headers
    options.headers = {
        ...options.headers,
        ...getAuthHeaders()
    };

    // 2. 发起请求
    const res = await fetch(url, options);
    const ct = (res.headers.get('content-type') || '').toLowerCase();

    // 3. 尝试解析 JSON
    if (ct.includes('application/json')) {
        try { return await res.json(); } catch (_) {}
    }

    // 4. 处理错误文本
    let txt = '';
    try { txt = await res.text(); } catch (_) {}
    return {
        status: 'error',
        message: `HTTP ${res.status}: ` + (txt ? txt.slice(0, 100) : 'Unknown Error')
    };
}

// --- 认证 API ---
export async function login(username, password) {
    const fd = new FormData();
    fd.append('username', username);
    fd.append('password', password);
    return await fetchJsonSafe('/api/auth/login', { method: 'POST', body: fd });
}

export async function register(username, password) {
    const fd = new FormData();
    fd.append('username', username);
    fd.append('password', password);
    return await fetchJsonSafe('/api/auth/register', { method: 'POST', body: fd });
}

// --- 管理员 API ---
export async function fetchAdminStats() {
    return await fetchJsonSafe('/api/admin/stats');
}

// --- 任务状态 API ---
export async function fetchStatus() {
    return await fetchJsonSafe('/api/status');
}

export async function fetchPlanners() {
    return await fetchJsonSafe('/api/planners');
}

// --- 文件操作 API ---
export async function fetchList(cat) {
    return await fetchJsonSafe(`/api/list/${encodeURIComponent(cat)}`);
}

export async function fetchProfile(search = '') {
    const query = search ? `?search=${encodeURIComponent(search)}` : '';
    return await fetchJsonSafe(`/api/user/profile${query}`);
}

export async function updateProfile(payload = {}) {
    const fd = new FormData();
    fd.append('new_username', payload.new_username || '');
    fd.append('display_name', payload.display_name || '');
    fd.append('email', payload.email || '');
    fd.append('phone', payload.phone || '');
    fd.append('notes', payload.notes || '');
    return await fetchJsonSafe('/api/user/profile/update', { method: 'POST', body: fd });
}

export async function rescanProfile() {
    return await fetchJsonSafe('/api/user/profile/rescan', { method: 'POST' });
}

export async function uploadFile(cat, file) {
    const fd = new FormData();
    fd.append('file', file);
    return await fetchJsonSafe(`/api/upload/${encodeURIComponent(cat)}`, { method: 'POST', body: fd });
}

export async function deleteFile(cat, name) {
    const fd = new FormData();
    fd.append('category', cat);
    fd.append('filename', name);
    return await fetchJsonSafe('/api/manage/delete', { method: 'POST', body: fd });
}

export async function renameFile(cat, oldName, newName) {
    const fd = new FormData();
    fd.append('category', cat);
    fd.append('old_name', oldName);
    fd.append('new_name', newName);
    return await fetchJsonSafe('/api/manage/rename', { method: 'POST', body: fd });
}

export async function fetchVis(ep, params) {
    return await fetchJsonSafe(`/api/visualize/${encodeURIComponent(ep)}?${params}`);
}

export async function processVoxelize(pc, weightProfileId = null) {
    const fd = new FormData();
    fd.append('pc_filename', pc);
    if(weightProfileId) fd.append('weight_profile_id', weightProfileId);
    return await fetchJsonSafe('/api/process/voxelize', { method: 'POST', body: fd });
}

export async function fetchWeightEditablePoints(pointCloudName, limit = 120000) {
    return await fetchJsonSafe(
        `/api/weights/${encodeURIComponent(pointCloudName)}/editable-points?limit=${encodeURIComponent(limit)}`
    );
}

function pointCloudLodQuery(options = {}) {
    const query = new URLSearchParams();
    if(options.variant) query.set('variant', options.variant);
    if(options.profile_id || options.profileId) query.set('profile_id', options.profile_id || options.profileId);
    const text = query.toString();
    return text ? `?${text}` : '';
}

export async function fetchPointCloudLodStatus(pointCloudName, options = {}) {
    return await fetchJsonSafe(
        `/api/pointcloud-lod/${encodeURIComponent(pointCloudName)}/status${pointCloudLodQuery(options)}`
    );
}

export async function preparePointCloudLod(pointCloudName, options = {}) {
    return await fetchJsonSafe(`/api/pointcloud-lod/${encodeURIComponent(pointCloudName)}/prepare${pointCloudLodQuery(options)}`, {
        method: 'POST'
    });
}

export async function fetchWeightStatus(pointCloudName) {
    return await fetchJsonSafe(`/api/weights/${encodeURIComponent(pointCloudName)}/status`);
}

export async function previewWeight(pointCloudName, profile) {
    return await fetchJsonSafe(`/api/weights/${encodeURIComponent(pointCloudName)}/preview`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(profile || {})
    });
}

export async function saveWeightDraft(pointCloudName, profile) {
    return await fetchJsonSafe(`/api/weights/${encodeURIComponent(pointCloudName)}/draft`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(profile || {})
    });
}

export async function applyWeight(pointCloudName, profile) {
    return await fetchJsonSafe(`/api/weights/${encodeURIComponent(pointCloudName)}/apply`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(profile || {})
    });
}

export async function restoreWeight(pointCloudName) {
    return await fetchJsonSafe(`/api/weights/${encodeURIComponent(pointCloudName)}/restore`, {
        method: 'POST'
    });
}

export async function processRL(vox, planner = '语义加权贪心多焦段视点规划', constraints = {}) {
    const fd = new FormData();
    fd.append('voxel_filename', vox);
    fd.append('planner', planner);
    Object.entries(constraints || {}).forEach(([key, value]) => {
        if(value !== null && value !== undefined && value !== '') fd.append(key, value);
    });
    return await fetchJsonSafe('/api/process/rl', { method: 'POST', body: fd });
}

export async function compareRoutes(selections) {
    return await fetchJsonSafe('/api/compare/routes', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ selections })
    });
}

export async function planRoute(waypointFilename, options = {}) {
    const fd = new FormData();
    fd.append('waypoint_filename', waypointFilename);
    if(options.safety_distance_m !== undefined) fd.append('safety_distance_m', options.safety_distance_m);
    if(options.clearance_m !== undefined) fd.append('clearance_m', options.clearance_m);
    if(options.wire_clearance_m !== undefined) fd.append('wire_clearance_m', options.wire_clearance_m);
    if(options.task_tower_clearance_m !== undefined) fd.append('task_tower_clearance_m', options.task_tower_clearance_m);
    if(options.task_wire_clearance_m !== undefined) fd.append('task_wire_clearance_m', options.task_wire_clearance_m);
    if(options.entry_distance_m !== undefined) fd.append('entry_distance_m', options.entry_distance_m);
    return await fetchJsonSafe('/api/route/plan', { method: 'POST', body: fd });
}

export async function validateRoute(category, filename, options = {}) {
    const fd = new FormData();
    fd.append('category', category);
    fd.append('filename', filename);
    if(options.safety_distance_m !== undefined) fd.append('safety_distance_m', options.safety_distance_m);
    if(options.tower_clearance_m !== undefined) fd.append('tower_clearance_m', options.tower_clearance_m);
    if(options.wire_clearance_m !== undefined) fd.append('wire_clearance_m', options.wire_clearance_m);
    if(options.task_tower_clearance_m !== undefined) fd.append('task_tower_clearance_m', options.task_tower_clearance_m);
    if(options.task_wire_clearance_m !== undefined) fd.append('task_wire_clearance_m', options.task_wire_clearance_m);
    return await fetchJsonSafe('/api/route/validate', { method: 'POST', body: fd });
}

export async function exportWaypoint(filename) {
    const res = await fetch(`/api/export/waypoint?filename=${encodeURIComponent(filename)}`, {
        headers: getAuthHeaders()
    });
    if (!res.ok) {
        let message = `HTTP ${res.status}`;
        try {
            const js = await res.json();
            message = js.message || message;
        } catch (_) {}
        throw new Error(message);
    }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    return { status: 'success' };
}

export async function exportRoute(filename, category = 'algorithm_route') {
    const res = await fetch(`/api/export/route?category=${encodeURIComponent(category)}&filename=${encodeURIComponent(filename)}`, {
        headers: getAuthHeaders()
    });
    if (!res.ok) {
        let message = `HTTP ${res.status}`;
        try {
            const js = await res.json();
            message = js.message || message;
        } catch (_) {}
        throw new Error(message);
    }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    return { status: 'success' };
}

