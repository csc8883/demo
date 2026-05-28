import { state, resetState } from './state.js?v=3.3';
import * as API from './api.js?v=3.3';
import * as Scene from './scene.js?v=3.3';
import { ui } from './ui.js?v=3.3';

console.log("App initializing...");

// --- 1. 强制全局绑定 (Defensive Binding) ---
// 将这些函数直接挂载到 window，确保 HTML onclick 能找到它们
window.ui = ui;
window.handleAuth = handleAuth;
window.logout = () => location.reload(); // 保持硬刷新以清理状态
window.executeUpload = executeUpload;
window.handleFileSelect = handleFileSelect;
window.runVoxelization = runVoxelization;
window.runRL = runRL;
window.openAlgorithmMenu = openAlgorithmMenu;
window.renderPlanningPointCloudPicker = renderPlanningPointCloudPicker;

// 修复 toggleLayer: 对应 ui.js 中的 window.toggleLayer(cat, id, this)
window.toggleLayer = (cat, id, el) => {
    // el 是 checkbox 元素
    const checked = el ? el.checked : true;

    // 调用 Scene.js 中正确的函数名 toggleObjectVisibility
    if (Scene.toggleObjectVisibility) {
        Scene.toggleObjectVisibility(cat, id, checked);
        if (ui.layers && ui.layers.toggle) {
            ui.layers.toggle(cat, id, checked);
        }
    } else {
        console.error("Scene.toggleObjectVisibility not found");
    }
};

// 修复 removeLayer: 对应 ui.js 中的 window.removeLayer(cat, id)
window.removeLayer = (cat, id) => {
    // 调用 ui.js 中的 layers.remove，它内部会处理 Scene 的删除和 UI 更新
    ui.layers.remove(cat, id);
};

window.handleDelete = handleDelete;
window.handleRename = handleRename;
window.openComparePage = openComparePage;
window.closeComparePage = closeComparePage;
window.loadCompareOptions = loadCompareOptions;
window.runRouteComparison = runRouteComparison;
window.loadExportOptions = loadExportOptions;
window.toggleRouteExportPanel = toggleRouteExportPanel;
window.exportRouteFile = exportRouteFile;
window.exportWaypointFile = exportRouteFile;
window.loadRoutePlanOptions = loadRoutePlanOptions;
window.toggleRoutePlanPanel = toggleRoutePlanPanel;
window.planRouteFromWaypoint = planRouteFromWaypoint;
window.loadRouteValidationOptions = loadRouteValidationOptions;
window.toggleRouteValidationPanel = toggleRouteValidationPanel;
window.validateRouteFile = validateRouteFile;
window.scrollToLogin = scrollToLogin;
window.openUserCenter = openUserCenter;
window.closeUserCenter = closeUserCenter;
window.saveProfileChanges = saveProfileChanges;
window.handleProfileSearch = handleProfileSearch;
window.handleProfileDelete = handleProfileDelete;

console.log("Global functions bound.");

initPlannerSelector();
ui.dropdown.closeAll();

let compareMainChart = null;
let compareRouteChart = null;
let profileFilesCache = [];
let profileDataCache = null;
let plannerCatalog = [];

window.handleProfileRename = handleProfileRename;
window.openProfileUpload = openProfileUpload;
window.refreshUserProfileData = refreshUserProfileData;
window.loadProfileFile = loadProfileFile;

function escapeHtml(value = '') {
    return String(value ?? '')
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#39;');
}

function escapeForInline(value = '') {
    return String(value ?? '').replace(/\\/g, '\\\\').replace(/'/g, "\\'");
}

function formatProfileText(value, fallback = '暂无') {
    const text = `${value ?? ''}`.trim();
    return text || fallback;
}

function setCurrentUserDisplay(name) {
    const label = document.getElementById('current-user-display');
    if(label) label.innerText = name || '用户';
}

function getProfileAuthHeaders() {
    const headers = {};
    if (state.user?.name) headers['X-User-Name'] = state.user.name;
    if (state.user?.token) headers['X-Auth-Token'] = state.user.token;
    return headers;
}

async function fetchProfileCompat(search = '') {
    if (typeof API.fetchProfile === 'function') {
        return API.fetchProfile(search);
    }
    const query = search ? `?search=${encodeURIComponent(search)}` : '';
    const res = await fetch(`/api/user/profile${query}`, {
        headers: getProfileAuthHeaders()
    });
    try {
        return await res.json();
    } catch (_) {
        return { status: 'error', message: `HTTP ${res.status}` };
    }
}

function scrollToLogin() {
    const modal = document.getElementById('login-modal');
    const target = document.getElementById('auth-section');
    if(!modal || !target) return;
    modal.scrollTo({ top: target.offsetTop, behavior: 'smooth' });
}

// --- 2. 初始化 3D 场景 ---
const container = document.getElementById('canvas-container');
if(container) {
    try {
        if(typeof THREE !== 'undefined') {
            Scene.init3D();
        } else {
            console.error("THREE is not defined");
        }
    } catch(e) { console.error("3D Init Error:", e); }
}

// 初始化状态
resetState();

// --- 3. 业务逻辑 ---

async function handleAuth() {
    const u = document.getElementById('username').value;
    const p = document.getElementById('password').value;

    if(typeof API.login !== 'function') {
        alert("System Error: API.login not loaded. Please refresh.");
        console.error("API module:", API);
        return;
    }

    if(ui.auth.mode === 'login') {
        const res = await API.login(u, p);
        if(res.status === 'success') {
            // 关键：这里写入的 state 必须与 api.js 读取的 state 是同一个实例
            state.user = res.data;
            ui.dropdown.closeAll();
            document.getElementById('login-modal').classList.add('hidden');

            if(state.user.role === 'admin') {
                document.getElementById('admin-app').classList.remove('hidden');
                ui.admin.renderDashboard();
            } else {
                document.getElementById('user-app').classList.remove('hidden');
                setCurrentUserDisplay(state.user.display_name || state.user.name);
                ui.sidebar.collapse();
                loadExportOptions();
                loadUserProfile().catch((e) => console.warn('加载个人中心失败', e));
                requestAnimationFrame(() => {
                    if (Scene.resizeRenderer) Scene.resizeRenderer();
                    setTimeout(() => Scene.resizeRenderer && Scene.resizeRenderer(), 120);
                });
            }
        } else alert(res.message);
    } else {
        const res = await API.register(u, p);
        if(res.status === 'success') {
            alert('注册成功，请登录');
            ui.auth.switchTab('login');
        } else alert(res.message);
    }
}

async function initPlannerSelector() {
    const select = document.getElementById('planner-select');
    if (!select || !API.fetchPlanners) return;
    try {
        const res = await API.fetchPlanners();
        if (res.status !== 'success' || !Array.isArray(res.data)) return;

        plannerCatalog = res.data;
        select.innerHTML = '';
        res.data.forEach((p) => {
            const opt = document.createElement('option');
            opt.value = p.key;
            opt.textContent = p.name;
            if (p.description) {
                opt.title = p.description;
            }
            select.appendChild(opt);
        });
        select.addEventListener('change', renderPlannerConfig);
        renderPlannerConfig();
    } catch (e) {
        console.warn('加载算法列表失败，使用默认配置', e);
    }
}

function selectedPlannerMeta() {
    const select = document.getElementById('planner-select');
    const key = select ? select.value : '';
    return plannerCatalog.find((item) => item.key === key) || null;
}

function renderPlannerConfig() {
    const container = document.getElementById('planner-config-container');
    if(!container) return;
    const planner = selectedPlannerMeta();
    const params = planner?.parameters || [];
    if(!params.length) {
        container.innerHTML = '<div class="text-[11px] text-slate-400">当前算法使用默认配置。</div>';
        return;
    }
    container.innerHTML = params.map((param) => `
        <label class="block">
            <span class="text-[11px] text-slate-500 font-bold">${escapeHtml(param.label || param.key)}</span>
            <input
                class="planner-param mt-1 w-full text-xs border border-slate-200 rounded-lg p-2 bg-white outline-none focus:ring-2 focus:ring-green-500"
                data-key="${escapeHtml(param.key)}"
                type="${param.type || 'number'}"
                value="${param.default ?? ''}"
                min="${param.min ?? ''}"
                max="${param.max ?? ''}"
                step="${param.step ?? '1'}"
            >
        </label>
    `).join('');
}

function collectPlannerConstraints() {
    const constraints = {};
    document.querySelectorAll('.planner-param').forEach((input) => {
        const key = input.dataset.key;
        if(!key) return;
        const value = input.value;
        if(value === '') return;
        const normalizedValue = input.type === 'number' ? Number(value) : value;
        if(key === 'target_manual_ratio') {
            const ratio = Math.max(0.6, Math.min(0.8, Number(normalizedValue)));
            constraints.manual_ratio_min = Number(ratio.toFixed(2));
            constraints.manual_ratio_max = Number(ratio.toFixed(2));
            return;
        }
        constraints[key] = normalizedValue;
    });
    return constraints;
}

function renderPlanningPointCloudPicker() {
    const container = document.getElementById('planning-pointcloud-list');
    if(!container) return;
    const pointclouds = state.loadedAssets.pointcloud || [];
    if(!pointclouds.length) {
        container.innerHTML = '<div class="text-[11px] text-slate-400 py-2">当前页面还没有加载点云。</div>';
        return;
    }
    container.innerHTML = pointclouds.map((item) => `
        <label class="flex items-center gap-2 p-2 rounded-lg hover:bg-slate-50 cursor-pointer">
            <input type="checkbox" class="planning-pointcloud-option accent-green-600" value="${escapeHtml(item.id)}" checked>
            <span class="min-w-0 text-xs text-slate-700 truncate" title="${escapeHtml(item.label || item.id)}">${escapeHtml(item.label || item.id)}</span>
        </label>
    `).join('');
}

function getSelectedPlanningPointClouds() {
    const checked = [...document.querySelectorAll('.planning-pointcloud-option:checked')].map((el) => el.value);
    if(checked.length) return checked;
    return (state.loadedAssets.pointcloud || []).map((item) => item.id);
}

function openAlgorithmMenu() {
    renderPlanningPointCloudPicker();
    renderPlannerConfig();
    ui.dropdown.toggle('dd-calc');
}

function profileFileTypeLabel(category) {
    return {
        point_cloud: '点云',
        manual_route: '人工航线',
        algorithm_route: '算法航线',
        voxel: '体素',
        waypoint: '算法航点'
    }[category] || category;
}

function getFilteredProfileFiles() {
    const keyword = (document.getElementById('profile-search-input')?.value || '').trim().toLowerCase();
    const category = document.getElementById('profile-category-filter')?.value || '';
    return profileFilesCache.filter((item) => {
        const matchesKeyword = !keyword || (item.name || '').toLowerCase().includes(keyword);
        const matchesCategory = !category || item.category === category;
        return matchesKeyword && matchesCategory;
    });
}

function syncProfileSummary(profile) {
    const summary = document.getElementById('profile-file-summary');
    if(!summary || !profile) return;
    const counts = profile.file_counts || {};
    summary.innerHTML = `
        <span>点云 ${counts.point_cloud || 0}</span>
        <span>人工航线 ${counts.manual_route || 0}</span>
        <span>算法航线 ${counts.algorithm_route || 0}</span>
        <span>体素 ${counts.voxel || 0}</span>
        <span>算法航点 ${counts.waypoint || 0}</span>
    `;

    const totalFiles = document.getElementById('profile-total-files');
    const totalSize = document.getElementById('profile-total-size');
    const lastLogin = document.getElementById('profile-last-login');
    const createdAt = document.getElementById('profile-created-at');
    const roleEl = document.getElementById('profile-role-label');

    if(totalFiles) totalFiles.innerText = `${profile.total_files || 0}`;
    if(totalSize) totalSize.innerText = profile.total_size || '0.00 MB';
    if(lastLogin) lastLogin.innerText = formatProfileText(profile.last_login, '首次使用');
    if(createdAt) createdAt.innerText = formatProfileText(profile.created_at, '暂无记录');
    if(roleEl) roleEl.textContent = profile.role === 'admin' ? '管理员' : '普通用户';
}

function renderProfileFiles(files) {
    const container = document.getElementById('profile-file-list');
    if(!container) return;
    if(!files.length) {
        container.innerHTML = `
            <div class="profile-file-empty">
                <i class="ph-duotone ph-folder-open text-4xl text-slate-300"></i>
                <p class="mt-3 text-sm text-slate-400">没有匹配的用户数据</p>
            </div>
        `;
        return;
    }

    container.innerHTML = files.map((item) => {
        const safeName = escapeHtml(item.name);
        const actionName = escapeForInline(item.name);
        const mtime = item.mtime ? new Date(item.mtime * 1000).toLocaleString() : '暂无记录';
        const extraMethod = item.extra?.method ? `<span class="profile-file-note">${escapeHtml(item.extra.method)}</span>` : '';
        return `
            <div class="profile-file-row">
                <div class="profile-file-main min-w-0">
                    <div class="flex items-center gap-2 min-w-0 flex-wrap">
                        <span class="profile-file-tag">${profileFileTypeLabel(item.category)}</span>
                        <span class="text-sm font-bold text-slate-700 truncate" title="${safeName}">${safeName}</span>
                        ${extraMethod}
                    </div>
                    <div class="text-xs text-slate-400 mt-1">${item.size || '--'} 路 ${mtime}</div>
                </div>
                <div class="profile-file-actions">
                    <button onclick="loadProfileFile('${item.category}', '${actionName}')" class="profile-action-btn text-blue-600">加载</button>
                    <button onclick="handleProfileDelete('${item.category}', '${actionName}')" class="profile-action-btn text-red-500">删除</button>
                </div>
            </div>
        `;
    }).join('');
}

async function loadUserProfile(search = '') {
    if(!state.user) return;
    const res = await fetchProfileCompat(search);
    if(res.status !== 'success') {
        throw new Error(res.message || '加载个人中心失败');
    }
    profileDataCache = res.data;
    profileFilesCache = Array.isArray(res.data.files) ? res.data.files : [];

    const usernameValue = document.getElementById('profile-username-value');
    const displayValue = document.getElementById('profile-display-value');
    const roleValue = document.getElementById('profile-role-text');
    if(usernameValue) usernameValue.innerText = res.data.username || '--';
    if(displayValue) displayValue.innerText = res.data.display_name || res.data.username || '--';
    if(roleValue) roleValue.innerText = res.data.role === 'admin' ? '管理员' : '普通用户';
    setCurrentUserDisplay(res.data.display_name || res.data.username);
    if(state.user) state.user.display_name = res.data.display_name || res.data.username;
    syncProfileSummary(res.data);
    renderProfileFiles(getFilteredProfileFiles());
}

async function openUserCenter() {
    ui.dropdown.closeAll();
    const modal = document.getElementById('profile-modal');
    if(modal) modal.classList.remove('hidden');
    const list = document.getElementById('profile-file-list');
    if(list) list.innerHTML = '<div class="text-sm text-slate-400 text-center py-10">正在加载个人中心...</div>';
    try {
        await loadUserProfile();
    } catch (e) {
        if(list) list.innerHTML = `<div class="text-sm text-red-500 text-center py-10">${e.message}</div>`;
    }
}

function closeUserCenter() {
    const modal = document.getElementById('profile-modal');
    if(modal) modal.classList.add('hidden');
}

function handleProfileSearch() {
    renderProfileFiles(getFilteredProfileFiles());
}

async function refreshUserProfileData() {
    const list = document.getElementById('profile-file-list');
    if(list) list.innerHTML = '<div class="text-sm text-slate-400 text-center py-10">正在刷新用户数据...</div>';
    const res = await API.rescanProfile();
    if(res.status !== 'success') {
        alert(res.message || '刷新失败');
        renderProfileFiles(getFilteredProfileFiles());
        return;
    }
    profileDataCache = res.data;
    profileFilesCache = Array.isArray(res.data.files) ? res.data.files : [];
    syncProfileSummary(res.data);
    renderProfileFiles(getFilteredProfileFiles());
}

function openProfileUpload() {
    const category = document.getElementById('profile-upload-category')?.value || 'point_cloud';
    ui.modals.openUpload(category);
}

async function saveProfileChanges() {
    const usernameInput = document.getElementById('profile-username-input');
    const displayInput = document.getElementById('profile-display-input');
    const emailInput = document.getElementById('profile-email-input');
    const phoneInput = document.getElementById('profile-phone-input');
    const notesInput = document.getElementById('profile-notes-input');
    if(!usernameInput) return;
    const nextUsername = (usernameInput.value || '').trim();
    const displayName = (displayInput?.value || '').trim();
    const email = (emailInput?.value || '').trim();
    const phone = (phoneInput?.value || '').trim();
    const notes = (notesInput?.value || '').trim();
    const res = await API.updateProfile({
        new_username: nextUsername,
        display_name: displayName,
        email,
        phone,
        notes
    });
    if(res.status !== 'success') {
        alert(res.message || '保存失败');
        return;
    }
    state.user.name = res.data.username;
    state.user.display_name = res.data.display_name || res.data.username;
    setCurrentUserDisplay(res.data.display_name || res.data.username);
    profileDataCache = res.data;
    profileFilesCache = Array.isArray(res.data.files) ? res.data.files : [];
    syncProfileSummary(res.data);
    renderProfileFiles(getFilteredProfileFiles());
    alert('个人中心已更新');
}

function loadProfileFile(category, filename) {
    closeUserCenter();
    handleFileSelect(category, filename);
}

async function handleProfileRename(category, oldName) {
    const newName = prompt(`请输入新的文件名（含扩展名）
原文件：${oldName}`, oldName);
    if(!newName || newName === oldName) return;
    const res = await API.renameFile(category, oldName, newName.trim());
    if(res.status !== 'success') {
        alert(res.message || '重命名失败');
        return;
    }
    const layerCategoryMap = {
        point_cloud: 'pointcloud',
        voxel: 'voxel',
        manual_route: 'route',
        algorithm_route: 'route',
        waypoint: 'route'
    };
    const layerCat = layerCategoryMap[category];
    if(layerCat && state.loadedAssets[layerCat]?.find(x => x.id === oldName)) {
        ui.layers.remove(layerCat, oldName);
    }
    await loadUserProfile();
    handleProfileSearch();
}

async function handleProfileDelete(category, filename) {
    if(!confirm(`确认删除 ${filename} 吗？`)) return;
    const res = await API.deleteFile(category, filename);
    if(res.status !== 'success') {
        alert(res.message || '删除失败');
        return;
    }
    const layerCategoryMap = {
        point_cloud: 'pointcloud',
        voxel: 'voxel',
        manual_route: 'route',
        algorithm_route: 'route',
        waypoint: 'route'
    };
    const layerCat = layerCategoryMap[category];
    if(layerCat && state.loadedAssets[layerCat]?.find(x => x.id === filename)) {
        ui.layers.remove(layerCat, filename);
    }
    await loadUserProfile();
    handleProfileSearch();
}

async function executeUpload() {
    const cat = document.getElementById('upload-category').value;
    const file = document.getElementById('upload-input').files[0];
    if(!file) return;

    if (ui.loading) ui.loading.start();
    try {
        const res = await API.uploadFile(cat, file);
        if(res.status === 'success') {
            alert('上传成功！');
            ui.modals.close('upload-modal');
            // 如果是管理员，刷新列表
            if(state.user && state.user.role === 'admin') {
                ui.admin.renderTable(cat);
            }
            const profileModal = document.getElementById('profile-modal');
            if(profileModal && !profileModal.classList.contains('hidden')) {
                await loadUserProfile();
                handleProfileSearch();
            }
        } else {
            alert("上传失败: " + res.message);
        }
    } catch(e) {
        alert("系统错误: " + e.message);
    } finally {
        if (ui.loading) ui.loading.stop();
    }
}

async function handleFileSelect(cat, filename) {
    if (ui.loading) ui.loading.start();
    try {
        if (cat === 'point_cloud') {
            const res = await API.fetchVis('pointcloud', new URLSearchParams({ filename }).toString());
            if (res.status === 'success') {
                ui.layers.clearSceneLayers({ resetCenter: true });
                Scene.renderPointCloud(res.data, filename);
                ui.layers.add('pointcloud', filename);
                renderPlanningPointCloudPicker();
                ui.led.set('pc', true);
            } else throw new Error(res.message);
        }
        else if (cat === 'manual_route' || cat === 'algorithm_route' || cat === 'waypoint') {
            const type = cat;
            const res = await API.fetchVis('result', new URLSearchParams({ type, filename }).toString());
            if (res.status === 'success') {
                const routeType = cat === 'manual_route' ? 'manual' : 'best';
                Scene.renderRoute(res.data.waypoints, filename, routeType);
                ui.layers.add('route', filename);
                ui.layers.updateMeta('route', filename, {
                    label: `${res.data.method_name || (cat === 'algorithm_route' ? '算法航线' : routeType === 'best' ? '最优航点' : '人工航线')} · ${filename}`,
                    method: res.data.method || (cat === 'algorithm_route' ? '算法航线' : routeType === 'best' ? '算法航点规划' : '人工航线'),
                    methodName: res.data.method_name || (cat === 'algorithm_route' ? '算法航线' : routeType === 'best' ? '最优航点' : '人工航线')
                });
                ui.sidebar.render(res.data.waypoints, filename, routeType);
                ui.sidebar.setRouteMeta(filename, res.data.method, res.data.method_name);
                ui.led.set('route', true);
            } else throw new Error(res.message);
        }
    } catch (e) {
        alert(e.message);
    } finally {
        if (ui.loading) ui.loading.stop();
    }
}

async function loadWaypointResult(filename, showAlert = false) {
    const res = await API.fetchVis('result', new URLSearchParams({ type: 'waypoint', filename }).toString());
    if (res.status !== 'success') {
        throw new Error(res.message || '加载强化学习结果失败');
    }

    Scene.renderRoute(res.data.waypoints, filename, 'best');
    ui.layers.add('route', filename);
    ui.layers.updateMeta('route', filename, {
        label: `${res.data.method_name || '语义表面体素分层强化学习多焦段视点规划'} · ${filename}`,
        method: res.data.method || '语义表面体素分层强化学习多焦段视点规划',
        methodName: res.data.method_name || '语义表面体素分层强化学习多焦段视点规划'
    });
    ui.sidebar.render(res.data.waypoints, filename, 'best');
    ui.sidebar.setRouteMeta(filename, res.data.method, res.data.method_name);
    ui.led.set('route', true);

    if (showAlert) alert(`规划完成，已自动加载结果：${filename}`);
    loadExportOptions();
}

async function runVoxelization() {
    const pointclouds = [...(state.loadedAssets.pointcloud || [])];
    if(!pointclouds.length) return alert("请先加载点云");

    for(const item of pointclouds) {
        const trigger = await API.processVoxelize(item.id);
        if (trigger.status !== 'success') return alert(`启动体素化失败：${trigger.message || '未知错误'}`);

        await ui.progress.start('voxelize', async () => {
            ui.led.set('vox', true);
            const base = item.id.replace(/\.[^.]+$/, '');
            const voxName = `${base}_voxel.npz`;
            const vRes = await API.fetchVis('result', new URLSearchParams({ type: 'voxel', filename: voxName }).toString());
            if(vRes.status === 'success') {
                const voxelLayerId = voxName;
                Scene.renderVoxels(vRes.data, vRes.data.center, voxelLayerId);
                ui.layers.add('voxel', voxelLayerId);
                ui.layers.updateMeta('voxel', voxelLayerId, { label: `栅格数据 · ${voxName}` });
            }
        });
    }
    alert(`已完成 ${pointclouds.length} 个点云的栅格化。`);
}

async function runRL() {
    const selectedPointClouds = getSelectedPlanningPointClouds();
    if(!selectedPointClouds.length) return alert("请先加载点云，并在航点管理中选择要规划的点云");

    const plannerSelect = document.getElementById('planner-select');
    const plannerKey = plannerSelect ? plannerSelect.value : '语义表面体素分层强化学习多焦段视点规划';
    const constraints = collectPlannerConstraints();

    for(const pointCloudName of selectedPointClouds) {
        const base = pointCloudName.replace(/\.[^.]+$/, '');
        const voxName = `${base}_voxel.npz`;
        const wpName = `${base}_${plannerKey}.json`;

        const trigger = await API.processRL(voxName, plannerKey, constraints);
        if (trigger.status !== 'success') {
            return alert(`启动 ${pointCloudName} 的 ${plannerKey} 失败：${trigger.message || '未知错误'}`);
        }

        await ui.progress.start('rl', async () => {
            try {
                await loadWaypointResult(wpName, false);
            } catch (e) {
                alert(`规划已完成，但自动加载结果失败：${e.message}`);
            }
        });
    }
    alert(`已完成 ${selectedPointClouds.length} 个点云的航点规划。`);
}

function setCompareVisible(visible) {
    const compare = document.getElementById('compare-page');
    const canvas = document.getElementById('canvas-container');
    const sidebar = document.getElementById('sidebar-panel');
    const sidebarBtn = document.getElementById('sidebar-expand-btn');
    const ledPanel = document.getElementById('status-led-panel');

    if(compare) compare.classList.toggle('hidden', !visible);
    if(canvas) canvas.classList.toggle('hidden', visible);
    if(sidebar) sidebar.classList.toggle('hidden', visible);
    if(sidebarBtn) sidebarBtn.classList.toggle('hidden', true);
    if(ledPanel) ledPanel.classList.toggle('hidden', visible);
    if(!visible && Scene.resizeRenderer) {
        requestAnimationFrame(() => Scene.resizeRenderer());
    }
}

async function openComparePage() {
    ui.dropdown.closeAll();
    setCompareVisible(true);
    await loadCompareOptions();
}

function closeComparePage() {
    setCompareVisible(false);
}

function routeOptionHtml(title, items, category) {
    if(!items.length) {
        return `<div class="text-xs text-slate-400 px-3 py-2">${title}暂无文件</div>`;
    }
    return `
        <div>
            <div class="text-xs font-bold text-slate-400 px-2 py-1">${title}</div>
            <div class="space-y-1">
                ${items.map((f) => `
                    <label class="flex items-start gap-2 p-2 rounded hover:bg-slate-50 cursor-pointer">
                        <input type="checkbox" class="compare-option mt-1 accent-blue-600" data-category="${category}" data-filename="${f.name}">
                        <span class="min-w-0">
                            <span class="block text-sm font-bold text-slate-700 truncate" title="${f.name}">${f.name}</span>
                            <span class="block text-xs text-slate-400">${f.owner || '当前用户'} · ${f.size}</span>
                        </span>
                    </label>
                `).join('')}
            </div>
        </div>
    `;
}

async function loadCompareOptions() {
    const container = document.getElementById('compare-options');
    if(!container || !state.user) return;
    container.innerHTML = '<div class="text-sm text-slate-400 p-6 text-center">正在加载可对比文件...</div>';
    try {
        const [manual, algorithmRoute, waypoint] = await Promise.all([
            API.fetchList('manual_route'),
            API.fetchList('algorithm_route'),
            API.fetchList('waypoint')
        ]);
        const manualItems = manual.status === 'success' ? manual.data : [];
        const algorithmRouteItems = algorithmRoute.status === 'success' ? algorithmRoute.data : [];
        const waypointItems = waypoint.status === 'success' ? waypoint.data : [];
        container.innerHTML =
            routeOptionHtml('人工航线', manualItems, 'manual_route') +
            routeOptionHtml('算法航线', algorithmRouteItems, 'algorithm_route') +
            routeOptionHtml('算法结果', waypointItems, 'waypoint');
    } catch (e) {
        container.innerHTML = `<div class="text-sm text-red-500 p-6 text-center">${e.message}</div>`;
    }
}

function fmtNumber(value, digits = 2) {
    if(value === null || value === undefined || Number.isNaN(Number(value))) return '--';
    return Number(value).toFixed(digits);
}

function fmtPercent(value) {
    if(value === null || value === undefined || Number.isNaN(Number(value))) return '--';
    return `${(Number(value) * 100).toFixed(1)}%`;
}

function destroyCompareCharts() {
    if(compareMainChart) compareMainChart.destroy();
    if(compareRouteChart) compareRouteChart.destroy();
    compareMainChart = null;
    compareRouteChart = null;
}

function statValue(stats, ...keys) {
    for(const key of keys) {
        const value = stats?.[key];
        if(value !== null && value !== undefined) return Number(value);
    }
    return null;
}

function metricBadge(text, level = '') {
    return level ? `<span class="${level}">${text}</span>` : text;
}

function updateCompareSummary(items) {
    const summary = document.getElementById('compare-summary');
    if(!summary) return;
    const valid = items.filter(x => !x.error);
    const algorithmItems = valid.filter(x => x.category === 'waypoint');
    const bestCoverage = algorithmItems.reduce((best, item) => {
        const cov = statValue(item.stats, 'coverage', 'coverage_weighted', 'C_weighted');
        return cov === null || cov === undefined ? best : Math.max(best, Number(cov));
    }, -1);
    const minSafety = algorithmItems.reduce((best, item) => {
        const value = statValue(item.stats, 'min_safety_distance_m');
        return value === null ? best : Math.min(best, value);
    }, Infinity);
    const bestTop = algorithmItems.reduce((best, item) => {
        const value = statValue(item.stats, 'C_top');
        return value === null ? best : Math.max(best, value);
    }, -1);
    const bestConnection = algorithmItems.reduce((best, item) => {
        const value = statValue(item.stats, 'C_connection_attention');
        return value === null ? best : Math.max(best, value);
    }, -1);

    const values = summary.querySelectorAll('.text-3xl');
    if(values[0]) values[0].textContent = bestCoverage >= 0 ? fmtPercent(bestCoverage) : '--';
    if(values[1]) values[1].textContent = Number.isFinite(minSafety) ? `${fmtNumber(minSafety)} m` : '--';
    if(values[2]) values[2].textContent = bestTop >= 0 ? fmtPercent(bestTop) : '--';
    if(values[3]) values[3].textContent = bestConnection >= 0 ? fmtPercent(bestConnection) : '--';

    const insights = document.getElementById('compare-insights');
    if(insights) {
        const lines = algorithmItems
            .map((item) => {
                const s = item.stats || {};
                const parts = [];
                const weighted = statValue(s, 'coverage', 'coverage_weighted', 'C_weighted');
                const ins = statValue(s, 'coverage_insulator', 'C_ins');
                const top = statValue(s, 'C_top');
                const edge = statValue(s, 'C_edge');
                const connection = statValue(s, 'C_connection_attention');
                const minSafetyValue = statValue(s, 'min_safety_distance_m');
                if(weighted !== null) parts.push(`加权 ${fmtPercent(weighted)}`);
                if(ins !== null) parts.push(`绝缘子 ${fmtPercent(ins)}`);
                if(top !== null) parts.push(`顶部 ${fmtPercent(top)}`);
                if(edge !== null) parts.push(`边缘 ${fmtPercent(edge)}`);
                if(connection !== null) parts.push(`连接关注 ${fmtPercent(connection)}`);
                if(minSafetyValue !== null) parts.push(`最小安全 ${fmtNumber(minSafetyValue)}m`);
                return parts.length ? `<div><span class="font-bold text-slate-700">${item.method_name || item.filename}</span>：${parts.join('；')}。</div>` : '';
            })
            .filter(Boolean);
        insights.innerHTML = lines.length
            ? `<div class="font-bold text-slate-700 mb-2">算法结果关键指标</div><div class="space-y-1">${lines.join('')}</div>`
            : '选择人工航点和算法结果后，这里会给出关键部件覆盖、安全距离和多焦段拍摄的对比提示。';
    }
}

function renderCompareCharts(items) {
    const valid = items.filter(x => !x.error);
    const labels = valid.map(x => x.method_name || x.filename);
    const weightedCoverages = valid.map(x => {
        const value = statValue(x.stats, 'coverage', 'coverage_weighted', 'C_weighted');
        return value === null ? null : value * 100;
    });
    const insulatorCoverages = valid.map(x => {
        const value = statValue(x.stats, 'coverage_insulator', 'C_ins');
        return value === null ? null : value * 100;
    });
    const topCoverages = valid.map(x => {
        const value = statValue(x.stats, 'C_top');
        return value === null ? null : value * 100;
    });
    const edgeCoverages = valid.map(x => {
        const value = statValue(x.stats, 'C_edge');
        return value === null ? null : value * 100;
    });
    const connectionCoverages = valid.map(x => {
        const value = statValue(x.stats, 'C_connection_attention');
        return value === null ? null : value * 100;
    });
    const counts = valid.map(x => Number(x.stats?.waypoint_count || x.stats?.count || 0));
    const shotCounts = valid.map(x => Number(x.stats?.shot_count || 0));
    const minSafety = valid.map(x => statValue(x.stats, 'min_safety_distance_m'));
    const shotEfficiency = valid.map(x => {
        const value = statValue(x.stats, 'coverage_per_shot');
        return value === null ? null : value * 100;
    });

    destroyCompareCharts();
    const mainCanvas = document.getElementById('compare-main-chart');
    const routeCanvas = document.getElementById('compare-route-chart');
    if(mainCanvas) {
        compareMainChart = new Chart(mainCanvas, {
            type: 'bar',
            data: {
                labels,
                datasets: [
                    { label: '加权覆盖（%）', data: weightedCoverages, backgroundColor: '#0f8f7a' },
                    { label: '绝缘子（%）', data: insulatorCoverages, backgroundColor: '#c7446f' },
                    { label: '塔顶（%）', data: topCoverages, backgroundColor: '#f97316' },
                    { label: '边缘结构（%）', data: edgeCoverages, backgroundColor: '#7c3aed' },
                    { label: '导/地线连接（%）', data: connectionCoverages, backgroundColor: '#2563eb' }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { position: 'bottom' }
                },
                scales: {
                    y: { beginAtZero: true, max: 100, position: 'left' }
                }
            }
        });
    }
    if(routeCanvas) {
        compareRouteChart = new Chart(routeCanvas, {
            type: 'bar',
            data: {
                labels,
                datasets: [
                    { label: '航点数', data: counts, backgroundColor: '#d88a18', yAxisID: 'y' },
                    { label: '拍摄次数', data: shotCounts, backgroundColor: '#7c3aed', yAxisID: 'y' },
                    { label: '最小安全距离(m)', data: minSafety, type: 'line', borderColor: '#dc2626', backgroundColor: '#dc2626', yAxisID: 'y1', tension: 0.25 },
                    { label: '单拍覆盖效率（%）', data: shotEfficiency, type: 'line', borderColor: '#0f8f7a', backgroundColor: '#0f8f7a', yAxisID: 'y1', tension: 0.25 }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { position: 'bottom' }
                },
                scales: {
                    y: { beginAtZero: true, position: 'left' },
                    y1: { beginAtZero: true, position: 'right', grid: { drawOnChartArea: false } }
                }
            }
        });
    }
}

function renderCompareTable(items) {
    const table = document.getElementById('compare-table');
    if(!table) return;
    const rows = items.map((item) => {
        if(item.error) {
            return `<tr><td class="p-3 text-red-500" colspan="15">${item.filename}: ${item.error}</td></tr>`;
        }
        const s = item.stats || {};
        const weighted = statValue(s, 'coverage', 'coverage_weighted', 'C_weighted');
        const ins = statValue(s, 'coverage_insulator', 'C_ins');
        const top = statValue(s, 'C_top');
        const edge = statValue(s, 'C_edge');
        const body = statValue(s, 'C_body');
        const connection = statValue(s, 'C_connection_attention');
        const minSafety = statValue(s, 'min_safety_distance_m');
        const safetyLimit = Number(s.safety_distance_m || 5);
        const safetyViolation = Number(s.safety_violation_count || 0);
        const shotCount = s.shot_count ?? '--';
        const focalUsage = s.focal_usage ? Object.entries(s.focal_usage).map(([k, v]) => `${k}:${v}`).join(' / ') : '--';
        const safetyClass = safetyViolation > 0 || (minSafety !== null && minSafety + 1e-9 < safetyLimit) ? 'metric-danger' : '';
        const topClass = top !== null && top < 0.94 ? 'metric-warning' : '';
        const connectionClass = connection !== null && connection < 0.50 ? 'metric-warning' : '';
        return `
            <tr class="border-b hover:bg-slate-50">
                <td class="p-3 font-bold text-slate-700">${item.method_name || item.method}</td>
                <td class="p-3 text-slate-500">${item.filename}</td>
                <td class="p-3">${fmtPercent(weighted)}</td>
                <td class="p-3">${fmtPercent(ins)}</td>
                <td class="p-3">${metricBadge(fmtPercent(top), topClass)}</td>
                <td class="p-3">${fmtPercent(edge)}</td>
                <td class="p-3">${fmtPercent(body)}</td>
                <td class="p-3">${metricBadge(fmtPercent(connection), connectionClass)}</td>
                <td class="p-3">${s.waypoint_count ?? s.count ?? '--'}</td>
                <td class="p-3">${shotCount}</td>
                <td class="p-3">${metricBadge(`${fmtNumber(minSafety)} m`, safetyClass)}</td>
                <td class="p-3">${metricBadge(String(safetyViolation), safetyClass)}</td>
                <td class="p-3">${s.covered_count ?? '--'} / ${s.target_count ?? '--'}</td>
                <td class="p-3">${focalUsage}</td>
            </tr>
        `;
    }).join('');
    table.innerHTML = `
        <table class="w-full text-xs text-left">
            <thead class="bg-slate-50 text-slate-500 font-bold text-xs">
                <tr>
                    <th class="p-3">方案</th>
                    <th class="p-3">文件</th>
                    <th class="p-3">加权覆盖</th>
                    <th class="p-3">绝缘子</th>
                    <th class="p-3">塔顶</th>
                    <th class="p-3">边缘</th>
                    <th class="p-3">塔身</th>
                    <th class="p-3">连接关注</th>
                    <th class="p-3">航点数</th>
                    <th class="p-3">拍摄数</th>
                    <th class="p-3">最小安全</th>
                    <th class="p-3">安全违规</th>
                    <th class="p-3">覆盖目标</th>
                    <th class="p-3">焦距使用</th>
                </tr>
            </thead>
            <tbody>${rows}</tbody>
        </table>
    `;
}

async function runRouteComparison() {
    const checked = [...document.querySelectorAll('.compare-option:checked')];
    const selections = checked.map((el) => ({
        category: el.dataset.category,
        filename: el.dataset.filename
    }));
    if(!selections.length) return alert('请先选择需要对比的航点文件');

    const res = await API.compareRoutes(selections);
    if(res.status !== 'success') return alert(res.message || '对比失败');
    const items = res.data.items || [];
    updateCompareSummary(items);
    renderCompareCharts(items);
    renderCompareTable(items);
}

async function loadExportOptions() {
    const container = document.getElementById('export-list-container');
    if(!container || !state.user) return;
    container.innerHTML = '<div class="text-xs text-slate-400 text-center py-2">正在加载算法航线...</div>';
    try {
        const res = await API.fetchList('algorithm_route');
        if(res.status !== 'success') throw new Error(res.message || '加载失败');
        if(!res.data.length) {
            container.innerHTML = '<div class="text-xs text-slate-400 text-center py-2 italic">暂无可导出的算法航线</div>';
            return;
        }
        container.innerHTML = res.data.map((f) => `
            <button onclick="exportRouteFile('${f.name.replace(/'/g, "\\'")}')" class="w-full text-left p-2 rounded hover:bg-slate-50 transition flex items-center justify-between gap-2">
                <span class="min-w-0">
                    <span class="block text-sm font-bold text-slate-700 truncate" title="${f.name}">${f.name}</span>
                    <span class="block text-xs text-slate-400">${f.size} · ${f.owner || '当前用户'}</span>
                </span>
                <i class="ph-bold ph-download-simple text-blue-600"></i>
            </button>
        `).join('');
    } catch(e) {
        container.innerHTML = `<div class="text-xs text-red-500 text-center py-2">${e.message}</div>`;
    }
}

async function toggleRouteExportPanel() {
    const container = document.getElementById('export-list-container');
    if(!container) return;
    const willOpen = container.classList.contains('hidden');
    container.classList.toggle('hidden', !willOpen);
    if(willOpen) await loadExportOptions();
}

function numericInputValue(id, fallback) {
    const el = document.getElementById(id);
    const value = Number(el?.value);
    return Number.isFinite(value) ? value : fallback;
}

function routeSafetySettingsHtml(prefix) {
    return `
        <div class="rounded-lg border border-slate-200 bg-white p-2 mb-2">
            <div class="text-[11px] font-bold text-slate-500 mb-2">安全距离参数(m)</div>
            <div class="grid grid-cols-1 gap-2 text-xs">
                <label class="block">
                    <span class="block text-slate-400 mb-1">安全距离</span>
                    <input id="${prefix}-safety-distance" type="number" min="0.5" max="40" step="0.1" value="5" class="w-full border border-slate-200 rounded px-2 py-1">
                </label>
            </div>
        </div>
    `;
}

function getRouteSafetyOptions(prefix) {
    return {
        safety_distance_m: numericInputValue(`${prefix}-safety-distance`, 5),
    };
}

async function loadRoutePlanOptions() {
    const container = document.getElementById('route-plan-list-container');
    if(!container || !state.user) return;
    container.innerHTML = '<div class="text-xs text-slate-400 text-center py-2">正在加载算法航点...</div>';
    try {
        const res = await API.fetchList('waypoint');
        if(res.status !== 'success') throw new Error(res.message || '加载失败');
        if(!res.data.length) {
            container.innerHTML = routeSafetySettingsHtml('route-plan') + '<div class="text-xs text-slate-400 text-center py-2 italic">暂无可规划的算法航点</div>';
            return;
        }
        container.innerHTML = routeSafetySettingsHtml('route-plan') + res.data.map((f) => `
            <button onclick="planRouteFromWaypoint('${f.name.replace(/'/g, "\\'")}')" class="w-full text-left p-2 rounded hover:bg-purple-50 transition flex items-center justify-between gap-2">
                <span class="min-w-0">
                    <span class="block text-sm font-bold text-slate-700 truncate" title="${f.name}">${f.name}</span>
                    <span class="block text-xs text-slate-400">${f.size} · 按上方安全距离生成到算法航线目录</span>
                </span>
                <i class="ph-bold ph-path text-purple-600"></i>
            </button>
        `).join('');
    } catch(e) {
        container.innerHTML = `<div class="text-xs text-red-500 text-center py-2">${e.message}</div>`;
    }
}

async function toggleRoutePlanPanel() {
    const container = document.getElementById('route-plan-list-container');
    if(!container) return;
    const willOpen = container.classList.contains('hidden');
    container.classList.toggle('hidden', !willOpen);
    if(willOpen) await loadRoutePlanOptions();
}

function routeValidationOptionHtml(title, items, category) {
    if(!items.length) return `<div class="text-xs text-slate-400 px-2 py-2">${title}暂无文件</div>`;
    return `
        <div class="mb-2">
            <div class="text-xs font-bold text-slate-400 px-2 py-1">${title}</div>
            ${items.map((f) => `
                <button onclick="validateRouteFile('${category}', '${f.name.replace(/'/g, "\\'")}')" class="w-full text-left p-2 rounded hover:bg-green-50 transition flex items-center justify-between gap-2">
                    <span class="min-w-0">
                        <span class="block text-sm font-bold text-slate-700 truncate" title="${f.name}">${f.name}</span>
                        <span class="block text-xs text-slate-400">${f.size} · 按上方安全距离校验</span>
                    </span>
                    <i class="ph-bold ph-shield-check text-green-600"></i>
                </button>
            `).join('')}
        </div>
    `;
}

async function loadRouteValidationOptions() {
    const container = document.getElementById('route-validation-list-container');
    if(!container || !state.user) return;
    container.innerHTML = '<div class="text-xs text-slate-400 text-center py-2">正在加载航线...</div>';
    try {
        const [manual, algorithm] = await Promise.all([
            API.fetchList('manual_route'),
            API.fetchList('algorithm_route')
        ]);
        const manualItems = manual.status === 'success' ? manual.data : [];
        const algorithmItems = algorithm.status === 'success' ? algorithm.data : [];
        container.innerHTML =
            routeSafetySettingsHtml('route-validation') +
            routeValidationOptionHtml('人工航线', manualItems, 'manual_route') +
            routeValidationOptionHtml('算法航线', algorithmItems, 'algorithm_route');
    } catch(e) {
        container.innerHTML = `<div class="text-xs text-red-500 text-center py-2">${e.message}</div>`;
    }
}

async function toggleRouteValidationPanel() {
    const container = document.getElementById('route-validation-list-container');
    if(!container) return;
    const willOpen = container.classList.contains('hidden');
    container.classList.toggle('hidden', !willOpen);
    if(willOpen) await loadRouteValidationOptions();
}

async function validateRouteFile(category, filename) {
    if (ui.loading) ui.loading.start();
    try {
        const res = await API.validateRoute(category, filename, getRouteSafetyOptions('route-validation'));
        if(res.status !== 'success') throw new Error(res.message || '安全距离校验失败');
        const data = res.data || {};
        const status = data.passed ? '通过' : '未通过';
        const detail = [
            `校验结果：${status}`,
            `安全距离：${data.safety_distance_m ?? data.task_tower_clearance_m ?? '--'} m`,
            `任务点：${data.task_point_count ?? '--'} 个`,
            `辅助/出入塔点：${data.auxiliary_point_count ?? '--'} 个`,
            `杆塔最小距离：${data.min_tower_distance_m ?? '--'} m`,
            `导线最小距离：${data.min_wire_distance_m ?? '--'} m`,
            `导线禁飞体：${data.conductor_no_fly_volume_count ?? 0} 个（最小余量 ${data.min_conductor_no_fly_clearance_m ?? '--'} m）`,
            `违规数量：${data.violation_count || 0}（任务点 ${data.task_violation_count || 0} / 辅助点 ${data.auxiliary_violation_count || 0} / 航段 ${data.segment_violation_count || 0}）`,
            data.voxel_file ? `体素文件：${data.voxel_file}` : '未找到匹配体素文件'
        ];
        if(data.violations && data.violations.length) {
            const sample = data.violations.slice(0, 5).map((v) => {
                const target = v.target === 'wire' ? '导线' : (v.target === 'conductor_no_fly' ? '导线禁飞体' : '杆塔');
                const where = v.type === 'segment' ? `${v.from}-${v.to}段` : `${v.index}号点`;
                return `${where} 距${target} ${v.distance_m}m（阈值 ${v.threshold_m ?? '--'}m）`;
            }).join('\n');
            detail.push(`前5条违规：\n${sample}`);
        }
        alert(detail.join('\n'));
    } catch(e) {
        alert(e.message);
    } finally {
        if (ui.loading) ui.loading.stop();
    }
}

async function planRouteFromWaypoint(filename) {
    if (ui.loading) ui.loading.start();
    try {
        const res = await API.planRoute(filename, getRouteSafetyOptions('route-plan'));
        if(res.status !== 'success') throw new Error(res.message || '航线规划失败');
        const out = res.data.filename;
        await handleFileSelect('algorithm_route', out);
        await loadUserProfile().catch(() => {});
        const c = res.data.clearance || {};
        alert(`航线规划完成：${out}\n航线点数：${res.data.route_point_count}\n航程：${res.data.totalLen} m\nA*局部规划段：${res.data.astar_segment_count ?? 0}，失败回退：${res.data.astar_fallback_count ?? 0}\n安全距离：${c.safety_distance_m ?? c.task_tower_clearance_m ?? '--'}m\n导线禁飞体：${c.conductor_no_fly_volume_count ?? 0} 个`);
    } catch(e) {
        alert(e.message);
    } finally {
        if (ui.loading) ui.loading.stop();
    }
}

async function exportRouteFile(filename) {
    try {
        await API.exportRoute(filename, 'algorithm_route');
    } catch(e) {
        alert(`导出失败：${e.message}`);
    }
}

// 3D 点选航点后，同步高亮/展开左侧列表
window.addEventListener('waypoint-click', (e) => {
    if (!e || !e.detail || !e.detail.id) return;
    ui.sidebar.expandAndHighlight(e.detail.id, e.detail.fileId || null);
});

async function handleDelete(cat, name) {
    if(confirm(`确认删除 ${name}?`)) {
        await API.deleteFile(cat, name);
        if(state.user.role === 'admin') ui.admin.renderTable(cat);
    }
}

async function handleRename(cat, oldName) {
    const newName = prompt(`请输入新的文件名（含扩展名）
原文件：${oldName}`, oldName);
    if(!newName || newName === oldName) return;

    const res = await API.renameFile(cat, oldName, newName);
    if(res.status === 'success') {
        if(state.user.role === 'admin') ui.admin.renderTable(cat);
    } else {
        alert(res.message);
    }
}

