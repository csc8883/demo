import * as API from './api.js?v=3.3';
import { state, resetCoordinateCenter } from './state.js?v=3.3';
import * as Scene from './scene.js?v=3.3';

const $ = (id) => document.getElementById(id);
const routeDomKey = (filename) => encodeURIComponent(String(filename ?? ''));
const waypointItemId = (filename, waypointId) => `wp-item-${routeDomKey(filename)}-${waypointId}`;
const waypointDetailId = (filename, waypointId) => `wp-detail-${routeDomKey(filename)}-${waypointId}`;

export const ui = {
    // --- 新增: 加载遮罩 (修复报错的关键) ---
    loading: {
        el: null,
        ensure() {
            if(this.el) return;
            this.el = document.createElement('div');
            this.el.id = 'global-loading';
            this.el.className = 'fixed inset-0 bg-slate-900/50 z-[100] flex items-center justify-center hidden backdrop-blur-sm';
            this.el.innerHTML = '<div class="bg-white p-6 rounded-2xl shadow-2xl flex flex-col items-center gap-3"><i class="ph-duotone ph-spinner animate-spin text-4xl text-blue-600"></i><span class="text-sm font-bold text-slate-600">处理中...</span></div>';
            document.body.appendChild(this.el);
        },
        start() {
            this.ensure();
            this.el.classList.remove('hidden');
        },
        stop() {
            if(this.el) this.el.classList.add('hidden');
        }
    },

    // --- 新增: 状态指示灯 ---
    led: {
        set(id, active) {
            const el = $(`led-${id}`);
            if(el) {
                if(active) {
                    el.classList.add('active');
                    el.style.backgroundColor = '#22c55e'; // Tailwind green-500
                    el.style.boxShadow = '0 0 10px #22c55e';
                } else {
                    el.classList.remove('active');
                    el.style.backgroundColor = '#cbd5e1'; // Tailwind slate-300
                    el.style.boxShadow = 'none';
                }
            }
        }
    },

    // --- 新增: 进度条控制 ---
    progress: {
        timer: null,
        async start(task, onComplete) {
            const modal = $('progress-modal');
            const bar = $('prog-bar');
            const text = $('prog-text');
            const name = $('prog-task-name');

            if(modal) modal.classList.remove('hidden');
            if(name) name.innerText = task === 'voxelize' ? '体素化处理中...' : '路径规划中...';
            if(bar) bar.style.width = '0%';
            if(text) text.innerText = '0%';

            if(this.timer) clearInterval(this.timer);

            return new Promise((resolve, reject) => {
                this.timer = setInterval(async () => {
                    try {
                        // 需要 api.js 支持 fetchStatus
                        if(API.fetchStatus) {
                            const res = await API.fetchStatus();
                            if(res.status === 'success') {
                                const info = res.data[task];
                                if(info) {
                                    const p = info.progress;
                                    if(bar) bar.style.width = `${p}%`;
                                    if(text) text.innerText = `${p}%`;

                                    if(info.status === 'error') {
                                        clearInterval(this.timer);
                                        if(modal) modal.classList.add('hidden');
                                        const err = new Error(info.message || '任务执行失败');
                                        alert(err.message);
                                        reject(err);
                                        return;
                                    }

                                    if(p >= 100 || info.status === 'completed') {
                                        clearInterval(this.timer);
                                        setTimeout(async () => {
                                            try {
                                                if(modal) modal.classList.add('hidden');
                                                if(onComplete) await onComplete();
                                                resolve();
                                            } catch (e) {
                                                reject(e);
                                            }
                                        }, 500);
                                    }
                                }
                            }
                        } else {
                            // 如果没有 API，模拟进度（防止卡死）
                            console.warn("fetchStatus API not found, faking progress");
                            clearInterval(this.timer);
                            if(modal) modal.classList.add('hidden');
                            if(onComplete) await onComplete();
                            resolve();
                        }
                    } catch(e) {
                        console.error("Poll error", e);
                    }
                }, 1000);
            });
        }
    },

    auth: {
        mode: 'login', // login | register
        switchTab(mode) {
            this.mode = mode;
            const btn = $('auth-btn');
            const title = $('auth-title');

            const tLogin = $('tab-login');
            const tReg = $('tab-register');

            if(tLogin) tLogin.className = mode === 'login'
                ? 'flex-1 pb-2 font-bold text-blue-600 border-b-2 border-blue-600'
                : 'flex-1 pb-2 font-bold text-slate-400 hover:text-slate-600';

            if(tReg) tReg.className = mode === 'register'
                ? 'flex-1 pb-2 font-bold text-blue-600 border-b-2 border-blue-600'
                : 'flex-1 pb-2 font-bold text-slate-400 hover:text-slate-600';

            if(title) title.innerText = mode === 'login' ? '系统登录' : '用户注册';
            if(btn) btn.innerText = mode === 'login' ? '进入系统' : '立即注册';
        }
    },

    sidebar: {
        routeStore: {},

        collapse() {
            const sb = $('sidebar-panel');
            const btn = $('sidebar-expand-btn');
            if(sb) sb.classList.add('collapsed');
            if(btn) btn.classList.remove('hidden');
        },

        expand() {
            const sb = $('sidebar-panel');
            const btn = $('sidebar-expand-btn');
            if(sb) sb.classList.remove('collapsed');
            if(btn) btn.classList.add('hidden');
        },

        toggle() {
            const sb = $('sidebar-panel');
            const btn = $('sidebar-expand-btn');
            if(sb && btn) {
                if (sb.classList.contains('collapsed')) {
                    this.expand();
                } else {
                    this.collapse();
                }
            }
        },

        setRouteData(filename, wps, routeType = 'manual') {
            this.routeStore[filename] = {
                filename,
                waypoints: Array.isArray(wps) ? wps : [],
                routeType,
                method: routeType === 'best' ? '算法航点规划' : '人工航点',
                methodName: routeType === 'best' ? '算法航点规划' : '人工航点'
            };
            this.renderAll();
        },

        setRouteMeta(filename, method, methodName) {
            if (!this.routeStore[filename]) return;
            this.routeStore[filename].method = method || this.routeStore[filename].method;
            this.routeStore[filename].methodName = methodName || this.routeStore[filename].methodName;
            this.renderAll();
        },

        removeRouteData(filename) {
            delete this.routeStore[filename];
            this.renderAll();
        },

        render(wps, filename, routeType = 'manual') {
            this.setRouteData(filename, wps, routeType);
        },

        renderAll() {
            const c = $('waypoint-list');
            if (!c) return;

            const loadedRouteIds = new Set((state.loadedAssets.route || []).map((item) => item.id));
            const routeFiles = Object.values(this.routeStore).filter((routeFile) => loadedRouteIds.has(routeFile.filename));
            c.innerHTML = '';

            if (!routeFiles.length) {
                c.innerHTML = `
                    <div class="p-10 text-center text-slate-400 text-sm flex flex-col items-center justify-center h-full">
                        <i class="ph-duotone ph-map-trifold text-4xl mb-3 text-slate-300"></i>
                        <p>选择或计算航线以查看详情</p>
                    </div>
                `;
                return;
            }

            routeFiles.forEach((routeFile) => {
                const section = document.createElement('div');
                section.className = 'border-b border-slate-100';

                const kind = routeFile.routeType === 'best' ? '最优航点' : '人工航点';
                const badgeClass = routeFile.routeType === 'best'
                    ? 'bg-orange-100 text-orange-600'
                    : 'bg-purple-100 text-purple-600';
                const routeAsset = (state.loadedAssets.route || []).find(x => x.id === routeFile.filename);
                const visible = routeAsset ? !!routeAsset.visible : true;
                const visClass = visible ? 'bg-green-100 text-green-600' : 'bg-slate-200 text-slate-500';
                const visText = visible ? '可见' : '隐藏';

                const fileHeader = document.createElement('div');
                fileHeader.className = "bg-slate-100 p-2 text-xs font-bold text-slate-500 uppercase tracking-wider border-b flex items-center justify-between";
                fileHeader.innerHTML = `
                    <span><i class="ph-bold ph-file"></i> ${routeFile.filename}</span>
                    <span class="flex items-center gap-1">
                        <span class="px-2 py-0.5 rounded ${badgeClass}">${routeFile.methodName || kind}</span>
                        <span class="px-2 py-0.5 rounded ${visClass}">${visText}</span>
                    </span>
                `;
                section.appendChild(fileHeader);

                routeFile.waypoints.forEach((wp) => {
                    const item = document.createElement('div');
                    item.id = waypointItemId(routeFile.filename, wp.id);
                    item.className = 'waypoint-item border-b text-sm transition-colors duration-200';

                    const header = document.createElement('div');
                    header.className = 'p-3 hover:bg-slate-50 cursor-pointer font-bold flex justify-between items-center group';
                    header.innerHTML = `
                        <div class="flex items-center gap-2">
                            <span class="bg-slate-200 text-slate-600 w-6 h-6 rounded-full flex items-center justify-center text-xs group-hover:bg-blue-600 group-hover:text-white transition">${wp.id}</span>
                            <span>航点</span>
                        </div>
                        <span class="text-blue-500 font-normal text-xs px-2 py-0.5 bg-blue-50 rounded">${wp.action || 'fly'}</span>
                    `;

                    const detail = document.createElement('div');
                    detail.id = waypointDetailId(routeFile.filename, wp.id);
                    detail.className = 'hidden p-3 bg-slate-50 text-xs text-slate-600 space-y-1 pl-11';
                    const shots = Array.isArray(wp.shots) ? wp.shots : [];
                    const focalText = shots.length
                        ? [...new Set(shots.map((shot) => shot.focal_level).filter(Boolean))].join(' / ')
                        : (wp.focal_level || '--');
                    const focusText = shots.length
                        ? [...new Set(shots.map((shot) => shot.semantic_focus).filter(Boolean))].join(' / ')
                        : '--';
                    detail.innerHTML = `
                        <div class="grid grid-cols-2 gap-2">
                            <div><span class="text-slate-400">X:</span> ${(wp.pos_utm?.[0] || 0).toFixed(1)}</div>
                            <div><span class="text-slate-400">Y:</span> ${(wp.pos_utm?.[1] || 0).toFixed(1)}</div>
                            <div class="col-span-2"><span class="text-slate-400">Z:</span> ${(wp.pos_utm?.[2] || 0).toFixed(1)}</div>
                            <div><span class="text-slate-400">Pitch:</span> ${wp.pitch || 0}°</div>
                            <div><span class="text-slate-400">Yaw:</span> ${wp.yaw || 0}°</div>
                            <div><span class="text-slate-400">拍摄:</span> ${wp.shot_count || shots.length || 1} 次</div>
                            <div><span class="text-slate-400">焦距:</span> ${focalText || '--'}</div>
                            <div class="col-span-2"><span class="text-slate-400">关注:</span> ${focusText || '--'}</div>
                        </div>
                    `;

                    header.addEventListener('click', () => this.handleItemClick(routeFile.filename, wp, header, { toggleDetail: true, follow: true }));
                    item.appendChild(header);
                    item.appendChild(detail);
                    section.appendChild(item);
                });

                c.appendChild(section);
            });

        },

        setDetailVisibility(fileId, id, expanded) {
            const det = $(waypointDetailId(fileId, id));
            if (det) det.classList.toggle('hidden', !expanded);
        },

        handleItemClick(fileId, waypoint, el, options = {}) {
            const id = waypoint?.id;
            if (id === undefined || id === null) return;
            const { toggleDetail = true, follow = true } = options;
            const det = $(waypointDetailId(fileId, id));
            if (det) {
                const nextExpanded = toggleDetail ? det.classList.contains('hidden') : true;
                this.setDetailVisibility(fileId, id, nextExpanded);
            }

            document.querySelectorAll('.waypoint-item').forEach(i => i.classList.remove('selected'));
            if(el.closest('.waypoint-item')) el.closest('.waypoint-item').classList.add('selected');

            if(follow && Scene && Scene.flyTo) Scene.flyTo({ ...waypoint, fileId });
        },

        expandAndHighlight(id, fileId = null) {
            this.expand();
            const candidates = fileId
                ? [this.routeStore[fileId]].filter(Boolean)
                : Object.values(this.routeStore);
            for (const routeFile of candidates) {
                const waypoint = (routeFile?.waypoints || []).find((wp) => String(wp.id) === String(id));
                if (!waypoint) continue;
                const item = $(waypointItemId(routeFile.filename, waypoint.id));
                if (!item) continue;
                item.scrollIntoView({ behavior: 'smooth', block: 'center' });
                const header = item.querySelector('.cursor-pointer');
                if (header) this.handleItemClick(routeFile.filename, waypoint, header, { toggleDetail: false, follow: false });
                break;
            }
        }
    },

    layers: {
        normalizeCategory(cat) {
            if (cat === 'manual_route' || cat === 'waypoint') return 'route';
            if (cat === 'point_cloud') return 'pointcloud';
            return cat;
        },

        updateControl() {
            const c = $('layer-list-container');
            const counter = $('layer-count');

            const allAssets = [
                ...state.loadedAssets.pointcloud.map(x => ({...x, cat: 'pointcloud'})),
                ...state.loadedAssets.route.map(x => ({...x, cat: 'route'})),
                ...state.loadedAssets.voxel.map(x => ({...x, cat: 'voxel'}))
            ];

            if(counter) {
                counter.innerText = allAssets.length;
                counter.classList.toggle('hidden', allAssets.length === 0);
            }

            if(!c) return;

            if (allAssets.length === 0) {
                c.innerHTML = '<div class="text-xs text-slate-400 text-center py-2 italic">暂无加载图层</div>';
                return;
            }

            c.innerHTML = '';
            allAssets.forEach(asset => {
                const div = document.createElement('div');
                div.className = 'layer-item flex items-center justify-between p-2 rounded';

                let icon = 'ph-cube';
                if(asset.cat === 'pointcloud') icon = 'ph-cloud';
                if(asset.cat === 'route') icon = 'ph-path';

                const checked = asset.visible !== false;
                const title = asset.label || asset.id;
                const left = document.createElement('div');
                left.className = 'flex items-center gap-2 overflow-hidden';

                const checkbox = document.createElement('input');
                checkbox.type = 'checkbox';
                checkbox.checked = checked;
                checkbox.className = 'accent-blue-600 cursor-pointer';
                checkbox.addEventListener('change', () => window.toggleLayer(asset.cat, asset.id, checkbox));

                const iconEl = document.createElement('i');
                iconEl.className = `ph-bold ${icon} text-slate-400`;

                const titleEl = document.createElement('span');
                titleEl.className = 'text-sm truncate max-w-[120px]';
                titleEl.title = title;
                titleEl.textContent = title;

                left.appendChild(checkbox);
                left.appendChild(iconEl);
                left.appendChild(titleEl);

                const removeBtn = document.createElement('button');
                removeBtn.className = 'text-slate-300 hover:text-red-500 transition';
                removeBtn.innerHTML = '<i class="ph-bold ph-x"></i>';
                removeBtn.addEventListener('click', () => window.removeLayer(asset.cat, asset.id));

                div.appendChild(left);
                div.appendChild(removeBtn);
                c.appendChild(div);
            });
        },

        add(cat, id) {
            const key = this.normalizeCategory(cat);
            if (!state.loadedAssets[key].find(x => x.id === id)) {
                state.loadedAssets[key].push({ id, visible: true, label: id, method: null, methodName: null });
                this.updateControl();
            }
        },

        updateMeta(cat, id, meta = {}) {
            const key = this.normalizeCategory(cat);
            const list = state.loadedAssets[key] || [];
            const target = list.find(x => x.id === id);
            if (!target) return;
            Object.assign(target, meta);
            this.updateControl();
        },

        clearSceneLayers({ resetCenter = false } = {}) {
            ['pointcloud', 'route', 'voxel'].forEach((key) => {
                (state.loadedAssets[key] || []).forEach((asset) => {
                    if(Scene.removeObject) Scene.removeObject(key, asset.id);
                });
                state.loadedAssets[key] = [];
            });
            if (resetCenter) {
                resetCoordinateCenter();
            }
            ui.sidebar.routeStore = {};
            ui.sidebar.renderAll();
            this.updateControl();
            if(typeof window.renderPlanningPointCloudPicker === 'function') {
                window.renderPlanningPointCloudPicker();
            }
            ui.led.set('pc', false);
            ui.led.set('route', false);
            ui.led.set('vox', false);
        },

        remove(cat, id) {
            const key = this.normalizeCategory(cat);
            if(key === 'pointcloud') {
                this.clearSceneLayers({ resetCenter: true });
                return;
            }
            state.loadedAssets[key] = (state.loadedAssets[key] || []).filter(x => x.id !== id);
            if(Scene.removeObject) Scene.removeObject(key, id);
            this.updateControl();
            if(typeof window.renderPlanningPointCloudPicker === 'function') {
                window.renderPlanningPointCloudPicker();
            }
            if(key === 'route') {
                ui.sidebar.removeRouteData(id);
            }
        },

        toggle(cat, id, checked) {
            const key = this.normalizeCategory(cat);
            if (state.loadedAssets[key]) {
                const target = state.loadedAssets[key].find(x => x.id === id);
                if (target) target.visible = !!checked;
            }
            this.updateControl();
            if (key === 'route') {
                ui.sidebar.renderAll();
            }
        }
    },

    dropdown: {
        toggle(id) {
            document.querySelectorAll('.dropdown-menu').forEach(el => {
                if(el.id !== id) el.classList.add('hidden');
            });
            const el = $(id);
            if (el) el.classList.toggle('hidden');
        },
        closeAll() {
            document.querySelectorAll('.dropdown-menu').forEach(el => el.classList.add('hidden'));
        }
    },

    modals: {
        openUpload(cat) {
            ui.dropdown.closeAll();
            const inp = $('upload-category');
            if(inp) inp.value = cat;
            const m = $('upload-modal');
            if(m) m.classList.remove('hidden');
            const disp = $('upload-filename-display');
            if(disp) disp.innerText = '';
            const fileInp = $('upload-input');
            if(fileInp) fileInp.value = '';
        },
        close(id) {
            const el = $(id);
            if(el) el.classList.add('hidden');
        },
        async openList(cat, onSelectStr) {
            ui.dropdown.closeAll();
            const container = $('list-container');
            const modal = $('list-modal');
            const title = $('list-modal-title');

            if(title) {
                const map = {'point_cloud': '点云列表', 'manual_route': '人工航线列表', 'algorithm_route': '算法航线列表', 'waypoint': '规划航点列表'};
                title.innerText = map[cat] || '文件列表';
            }

            if(container) container.innerHTML = '<div class="p-10 text-center"><i class="ph-duotone ph-spinner animate-spin text-2xl text-blue-500"></i></div>';
            if(modal) modal.classList.remove('hidden');

            try {
                const res = await API.fetchList(cat);
                if (res.status !== 'success') throw new Error(res.message);

                const list = res.data;
                if(container) {
                    container.innerHTML = '';
                    if (!list.length) {
                        container.innerHTML = '<div class="p-10 text-center text-slate-400">暂无数据</div>';
                        return;
                    }

                    list.forEach(f => {
                        const div = document.createElement('div');
                        div.className = 'flex justify-between items-center p-4 border-b hover:bg-slate-50 cursor-pointer group transition';
                        div.innerHTML = `
                            <div class="flex flex-col">
                                <span class="font-bold text-slate-700 group-hover:text-blue-600 transition">${f.name}</span>
                                <span class="text-xs text-slate-400">${f.size} • ${new Date(f.mtime*1000).toLocaleString()}</span>
                            </div>
                            <div class="text-xs bg-slate-100 text-slate-500 px-2 py-1 rounded group-hover:bg-white group-hover:shadow">${f.owner || '当前用户'}</div>
                        `;
                        div.onclick = () => {
                            ui.modals.close('list-modal');
                            if(typeof window[onSelectStr] === 'function') window[onSelectStr](cat, f.name);
                        };
                        container.appendChild(div);
                    });
                }
            } catch (e) {
                if(container) container.innerHTML = `<div class="p-10 text-center text-red-500">${e.message}</div>`;
            }
        }
    },

    admin: {
        renderDashboard: async () => {
            const c = $('admin-content');
            if(!c) return;
            c.innerHTML = '<div class="p-10 text-center">正在加载统计数据...</div>';

            try {
                const res = await API.fetchAdminStats();
                if(res.status !== 'success') throw new Error(res.message);
                const s = res.data;

                c.innerHTML = `
                    <h2 class="text-2xl font-bold mb-6">系统概览</h2>
                    <div class="grid grid-cols-3 gap-6 mb-8">
                        <div class="bg-white p-6 rounded-xl shadow-sm border border-slate-100">
                            <div class="text-slate-500 text-xs font-bold uppercase">总用户数</div>
                            <div class="text-4xl font-bold text-blue-600 mt-2">${s.user_count}</div>
                        </div>
                        <div class="bg-white p-6 rounded-xl shadow-sm border border-slate-100">
                            <div class="text-slate-500 text-xs font-bold uppercase">总存储占用 (MB)</div>
                            <div class="text-4xl font-bold text-purple-600 mt-2">${s.total_size_mb}</div>
                        </div>
                        <div class="bg-white p-6 rounded-xl shadow-sm border border-slate-100">
                            <div class="text-slate-500 text-xs font-bold uppercase">文件总数</div>
                            <div class="text-4xl font-bold text-green-600 mt-2">${Object.values(s.file_counts).reduce((a,b)=>a+b,0)}</div>
                        </div>
                    </div>
                    
                    <div class="grid grid-cols-2 gap-6">
                        <div class="bg-white p-6 rounded-xl shadow-sm border border-slate-100">
                            <h3 class="font-bold mb-4 text-slate-700">用户存储分布 (MB)</h3>
                            <canvas id="chart-disk"></canvas>
                        </div>
                        <div class="bg-white p-6 rounded-xl shadow-sm border border-slate-100">
                            <h3 class="font-bold mb-4 text-slate-700">文件类型分布</h3>
                            <canvas id="chart-files"></canvas>
                        </div>
                    </div>
                `;

                new Chart($('chart-disk'), {
                    type: 'bar',
                    data: {
                        labels: Object.keys(s.disk_usage),
                        datasets: [{ label: '存储占用（MB）', data: Object.values(s.disk_usage), backgroundColor: '#0f8f7a' }]
                    }
                });

                new Chart($('chart-files'), {
                    type: 'doughnut',
                    data: {
                        labels: ['点云', '航线', '体素'],
                        datasets: [{
                            data: [s.file_counts.point_cloud, s.file_counts.route, s.file_counts.voxel],
                            backgroundColor: ['#0f8f7a', '#c7446f', '#d88a18']
                        }]
                    }
                });

            } catch(e) {
                c.innerHTML = `<div class="text-red-500">错误：${e.message}</div>`;
            }
        },

        renderTable: async (cat) => {
            const c = $('admin-content');
            if(!c) return;
            c.innerHTML = '<div class="p-10 text-center">正在加载...</div>';
            const res = await API.fetchList(cat);
            if(res.status !== 'success') { c.innerHTML = '加载失败'; return; }

            const titleMap = {
                point_cloud: '点云文件',
                manual_route: '人工航线',
                algorithm_route: '算法航线',
                voxel: '栅格数据',
                waypoint: '规划结果'
            };

            c.innerHTML = `
                <div class="flex justify-between items-center mb-6">
                    <h2 class="text-2xl font-bold">${titleMap[cat] || cat}</h2>
                    <button class="bg-blue-600 text-white px-4 py-2 rounded shadow hover:bg-blue-700" onclick="ui.modals.openUpload('${cat}')">上传文件</button>
                </div>
                <div class="bg-white rounded-xl shadow overflow-hidden">
                    <table class="w-full text-sm text-left">
                        <thead class="bg-slate-50 text-slate-500 font-bold uppercase text-xs">
                            <tr>
                                <th class="p-4">文件名</th>
                                <th class="p-4">所属用户</th>
                                <th class="p-4">大小</th>
                                <th class="p-4 text-right">操作</th>
                            </tr>
                        </thead>
                        <tbody class="divide-y">
                             ${res.data.map(f => `
                                <tr class="hover:bg-slate-50">
                                    <td class="p-4 font-medium text-slate-700">${f.name}</td>
                                    <td class="p-4 text-slate-500"><span class="bg-slate-100 px-2 py-1 rounded text-xs">${f.owner}</span></td>
                                    <td class="p-4 text-slate-400">${f.size}</td>
                                    <td class="p-4 text-right space-x-2">
                                        <button class="text-slate-400 hover:text-red-500" onclick="handleDelete('${cat}', '${f.name}')"><i class="ph-bold ph-trash"></i></button>
                                    </td>
                                </tr>
                             `).join('')}
                        </tbody>
                    </table>
                </div>
            `;
        }
    }
};
