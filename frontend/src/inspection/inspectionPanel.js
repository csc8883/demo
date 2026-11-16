import * as Scene from '../legacy/scene.js';
import { state } from '../legacy/state.js';
import { clone, componentOf, prepareWaypoints, editWaypoint } from './draftModel.js';
import { geometry, optics, targetInFrame } from './cameraModel.js';

const routes = new Map();
let currentFile = null, selectedId = null, shotId = null, filter = '', locked = true;
let panel, opened = false, gesture = null;
const el = id => document.getElementById(`inspection-${id}`);
const route = () => routes.get(currentFile);
const waypoint = () => route()?.working.find(w => w.waypoint_id === selectedId);
const shot = () => waypoint()?.shots.find(s => s.shot_id === shotId);
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const changed = r => JSON.stringify(r.working) !== JSON.stringify(r.original);
const field = (key, title, min, max, step, unit) => `<div class="inspection-field"><label for="inspection-${key}">${title}<small>${unit}</small></label><div class="inspection-control"><input aria-label="${title}滑块" data-field="${key}" type="range" min="${min}" max="${max}" step="${step}"><input id="inspection-${key}" aria-label="${title}" data-field="${key}" type="number" min="${min}" max="${max}" step="${step}"></div></div>`;

export function initInspectionPanel() {
    panel = document.createElement('aside'); panel.id = 'inspection-panel'; panel.className = 'inspection-panel hidden';
    panel.setAttribute('aria-label', '精细化航点编辑');
    panel.innerHTML = `
      <header class="inspection-heading"><div><small>DETAILED INSPECTION</small><h2>精细化航点</h2></div><button data-act="close" title="收起编辑面板" aria-label="收起编辑面板">×</button></header>
      <div class="inspection-scroll">
        <section class="inspection-source"><label for="inspection-route">规划结果</label><select id="inspection-route" aria-label="规划结果"></select><p id="inspection-coordinate">米制世界坐标 · Z 向上</p></section>
        <section class="inspection-selection"><label for="inspection-component">巡检部件</label><select id="inspection-component" aria-label="巡检部件"></select>
          <div class="inspection-waypoint-nav"><button data-act="prev" aria-label="上一个航点">‹</button><select id="inspection-waypoint" aria-label="当前航点"></select><button data-act="next" aria-label="下一个航点">›</button></div>
          <div id="inspection-counter" class="inspection-counter"></div>
          <label for="inspection-shot">拍摄动作</label><select id="inspection-shot" aria-label="拍摄动作"></select>
        </section>
        <section class="inspection-preview-section"><div class="inspection-section-title"><span>航点相机</span><span id="inspection-camera-badge">几何预览</span></div>
          <div class="inspection-preview"><canvas id="inspection-preview-canvas" width="360" height="240" aria-label="航点拍摄视角"></canvas><div class="inspection-reticle"></div><div id="inspection-preview-empty" class="inspection-preview-empty">加载点云以查看拍摄视角</div><span class="inspection-preview-label">CAM 01 <b>3:2</b></span></div>
          <p id="inspection-view-status" class="inspection-view-status" role="status"></p>
          <div class="inspection-camera-meta"><span id="inspection-fov">—</span><button data-act="focus">定位主视图 ↗</button></div>
        </section>
        <section id="inspection-parameters"><div class="inspection-section-title"><span>拍摄参数</span><label class="inspection-lock"><input id="inspection-lock" type="checkbox" checked>锁定目标</label></div>
          ${field('Distance', '拍摄距离', 1, 80, 0.1, 'm')}
          ${field('f_eq_mm', '等效焦距', 24, 200, 0.1, 'mm')}
          ${field('yaw', '航向角', 0, 360, 0.1, '°')}
          ${field('pitch', '云台俯仰', -90, 90, 0.1, '°')}
          <details class="inspection-position"><summary>航点位置 <span>世界坐标 / m</span></summary><div class="inspection-xyz">${['X', 'Y', 'Z'].map((axis, i) => `<label>${axis}<input type="number" step="0.1" data-field="pos_${i}" aria-label="航点位置${axis}"></label>`).join('')}</div></details>
          <p class="inspection-help">锁定目标时，调整距离与位置将保持镜头朝向目标。解除锁定后可自由调整航向和俯仰。</p>
        </section>
        <div id="inspection-transit" class="inspection-note hidden">过渡航点用于连接航线，没有拍摄动作。</div>
        <div id="inspection-message" class="inspection-note" role="status">原始结果 · 尚未修改</div>
      </div>
      <footer class="inspection-footer"><div><span id="inspection-draft-badge">原始结果</span><small>会话草稿 · 未重新评估</small></div><div class="inspection-buttons"><button data-act="undo" disabled>撤销</button><button data-act="reset" disabled>还原</button><button data-act="apply" class="primary" disabled>应用草稿</button></div></footer>`;
    document.querySelector('.workspace-body').prepend(panel);
    const reopen = document.createElement('button'); reopen.id = 'inspection-open'; reopen.className = 'inspection-open hidden';
    reopen.textContent = '精细化航点'; reopen.onclick = () => openPanel();
    document.getElementById('canvas-container').append(reopen);
    panel.addEventListener('click', e => action(e.target.closest('[data-act]')?.dataset.act));
    panel.addEventListener('input', onInput);
    panel.addEventListener('change', e => {
        if (!e.target.dataset.field) return;
        gesture = null;
        if (e.target.getAttribute('aria-invalid') === 'true') {
            const key = e.target.dataset.field, wp = waypoint(), s = shot();
            const value = key.startsWith('pos_') ? wp.pos_utm[Number(key.slice(4))] : key === 'Distance' && s.look_at ? geometry(wp.pos_utm, s.look_at).Distance : s[key];
            e.target.value = Number(value.toFixed(3));
            e.target.removeAttribute('aria-invalid');
            updateBadges('无效输入已恢复为上一次有效值。');
        }
    });
    panel.addEventListener('focusout', () => { gesture = null; });
    el('route').onchange = e => { currentFile = e.target.value; filter = ''; selectedId = null; renderOptions(); };
    el('component').onchange = e => { filter = e.target.value; selectedId = null; renderWaypoints(); };
    el('waypoint').onchange = e => select(e.target.value);
    el('shot').onchange = e => { shotId = e.target.value; locked = Boolean(shot()?.look_at); renderSelection(); };
    el('lock').onchange = e => {
        locked = e.target.checked;
        if (locked && shot()?.look_at) {
            const r = route(), wp = waypoint();
            const edited = editWaypoint(wp, shotId, 'pos_0', wp.pos_utm[0], true);
            if (JSON.stringify(edited) !== JSON.stringify(wp)) {
                r.history.push(clone(r.working)); r.working[r.working.indexOf(wp)] = edited;
                Scene.updateInspectionWaypoint(currentFile, edited); invalidateReports();
            }
        }
        renderSelection();
    };
    window.addEventListener('inspection-route-loaded', e => load(e.detail));
    window.addEventListener('inspection-suspend', () => closePanel());
    window.addEventListener('inspection-route-removed', e => remove(e.detail));
    window.addEventListener('inspection-clear', () => { routes.clear(); currentFile = null; closePanel(); elOpen().classList.add('hidden'); });
    window.addEventListener('waypoint-click', e => {
        if (!routes.has(e.detail.fileId)) return;
        currentFile = e.detail.fileId; filter = ''; selectedId = String(e.detail.waypoint?.waypoint_id ?? e.detail.id);
        renderOptions(); openPanel();
    });
    panel.addEventListener('keydown', e => {
        if (e.altKey && ['ArrowLeft', 'ArrowRight'].includes(e.key)) { e.preventDefault(); action(e.key === 'ArrowLeft' ? 'prev' : 'next'); }
    });
    new ResizeObserver(() => Scene.resizeRenderer()).observe(panel);
}
const elOpen = () => document.getElementById('inspection-open');
function load({ filename, waypoints, routeType }) {
    // Reopening an already loaded file must not silently throw away a session draft.
    if (routes.has(filename)) {
        currentFile = filename; selectedId = null; filter = '';
        routes.get(filename).working.forEach(w => Scene.updateInspectionWaypoint(filename, w));
        renderOptions(); openPanel(); return;
    }
    const prepared = prepareWaypoints(waypoints);
    if (!prepared.length) return;
    routes.set(filename, { original: clone(prepared), applied: clone(prepared), working: prepared, history: [], routeType });
    currentFile = filename; selectedId = null; filter = '';
    renderOptions(); openPanel();
}
function remove(filename) {
    routes.delete(filename);
    if (currentFile !== filename) return;
    currentFile = routes.keys().next().value || null; selectedId = null; filter = '';
    if (currentFile) renderOptions(); else { closePanel(); elOpen().classList.add('hidden'); }
}
function openPanel() {
    if (!route()) return;
    opened = true; panel.classList.remove('hidden'); elOpen().classList.add('hidden');
    // Avoid three simultaneous side panels squeezing the flight scene.
    document.getElementById('sidebar-panel')?.classList.add('collapsed');
    document.getElementById('layer-tree-panel')?.classList.add('collapsed');
    const frame = state.coordinateFrame;
    el('coordinate').textContent = frame?.crs ? `${frame.crs} · m · Z 向上` : '原始米制坐标 · Z 向上 · 坐标系未声明';
    el('coordinate').title = frame?.warning || '高程基准尚未确认；预览保持原始坐标';
    renderSelection(); Scene.resizeRenderer();
}
function closePanel() { opened = false; panel.classList.add('hidden'); elOpen().classList.toggle('hidden', !routes.size); Scene.setInspectionView(null); Scene.resizeRenderer(); }
function filtered() { return route()?.working.filter(w => !filter || w.shots.some(s => componentOf(s).key === filter)) || []; }
function renderOptions() {
    el('route').innerHTML = [...routes.keys()].map(k => `<option value="${esc(k)}">${esc(k.replace('.json', ''))}</option>`).join(''); el('route').value = currentFile;
    const components = new Map(); route().working.forEach(w => w.shots.forEach(s => { const c = componentOf(s); components.set(c.key, c); }));
    el('component').innerHTML = '<option value="">全部部件</option>' + [...components.values()].map(c => `<option value="${esc(c.key)}">${esc(c.name)}</option>`).join(''); el('component').value = filter;
    renderWaypoints();
}
function renderWaypoints() {
    const list = filtered();
    el('waypoint').innerHTML = list.map(w => `<option value="${esc(w.waypoint_id)}">航点 ${esc(w.id)} · ${w.shots.length ? `${w.shots.length} 个拍摄动作` : '过渡点'}</option>`).join('');
    select(list.some(w => w.waypoint_id === selectedId) ? selectedId : list[0]?.waypoint_id);
}
function select(id) {
    selectedId = id; gesture = null;
    const wp = waypoint();
    shotId = (filter ? wp?.shots.find(s => componentOf(s).key === filter) : wp?.shots[0])?.shot_id || wp?.shots[0]?.shot_id;
    locked = Boolean(shot()?.look_at); renderSelection();
}
function renderSelection() {
    const wp = waypoint(); if (!wp) return;
    const s = shot(); el('waypoint').value = selectedId;
    const list = filtered(), index = list.indexOf(wp);
    el('counter').textContent = `${index + 1} / ${list.length} 个航点${filter ? ' · 已按部件筛选' : ''}`;
    panel.querySelector('[data-act="prev"]').disabled = index <= 0;
    panel.querySelector('[data-act="next"]').disabled = index >= list.length - 1;
    el('shot').innerHTML = wp.shots.map((sh, i) => `<option value="${esc(sh.shot_id)}">${i + 1}. ${esc(componentOf(sh).name)}</option>`).join('');
    el('shot').value = shotId || ''; el('shot').disabled = !s;
    el('parameters').classList.toggle('hidden', !s); el('transit').classList.toggle('hidden', !!s);
    el('lock').checked = locked; el('lock').disabled = !s?.look_at;
    el('preview-empty').classList.toggle('hidden', !!s); el('preview-empty').textContent = s ? '正在准备视角…' : '该航点无拍摄动作';
    if (s) {
        for (const input of panel.querySelectorAll('[data-field]')) {
            const key = input.dataset.field;
            const value = key.startsWith('pos_') ? wp.pos_utm[Number(key.slice(4))] : key === 'Distance' && s.look_at ? geometry(wp.pos_utm, s.look_at).Distance : s[key];
            if (document.activeElement !== input) input.value = Number.isFinite(value) ? Number(value.toFixed(input.type === 'range' ? 1 : 3)) : '';
            input.disabled = (key === 'Distance' && (!locked || !s.look_at)) || (locked && ['yaw', 'pitch'].includes(key));
            input.removeAttribute('aria-invalid');
        }
        const fov = optics(s.f_eq_mm);
        el('fov').textContent = `FOV ${fov.hfov_deg.toFixed(1)}° × ${fov.vfov_deg.toFixed(1)}°`;
    }
    updateBadges();
    const component = s ? componentOf(s) : null;
    const componentTargets = component?.id ? route().working.flatMap(w => w.shots.filter(sh => componentOf(sh).id === component.id && sh.look_at).map(sh => sh.look_at)) : [];
    if (opened) Scene.setInspectionView({ waypoint: wp, shot: s, componentTargets, canvas: el('preview-canvas'), onStatus: previewStatus });
}
function previewStatus(status) {
    const s = shot(), wp = waypoint(); if (!s || !wp) return;
    const ready = status === 'ready' || status === 'voxel';
    el('preview-empty').classList.toggle('hidden', ready);
    if (!ready) el('preview-empty').textContent = status === 'error' ? '预览暂不可用，请重新选择航点' : '请加载与航点匹配的点云';
    const inside = targetInFrame(wp.pos_utm, s);
    el('view-status').textContent = !ready ? '等待场景数据' : inside === false ? '目标在画面外 · 调整朝向或锁定目标' : !s.look_at ? '未提供目标位置 · 可自由观察' : `${status === 'voxel' ? '体素预览' : '点云几何预览'} · 目标在视场内`;
    el('view-status').classList.toggle('warning', inside === false);
}
function updateBadges(message) {
    const r = route(); if (!r) return;
    const dirty = changed(r), pending = JSON.stringify(r.working) !== JSON.stringify(r.applied);
    el('draft-badge').textContent = pending ? '未应用草稿' : dirty ? '会话草稿已应用' : '原始结果';
    el('draft-badge').classList.toggle('dirty', dirty);
    el('message').textContent = message || (dirty ? '参数已变化，原覆盖率和安全报告已失效。应用仅更新会话草稿，不会覆盖算法文件。' : '原始结果 · 尚未修改');
    panel.querySelector('[data-act="undo"]').disabled = !r.history.length;
    panel.querySelector('[data-act="reset"]').disabled = !dirty;
    panel.querySelector('[data-act="apply"]').disabled = !pending;
}
function onInput(e) {
    const input = e.target, key = input.dataset.field; if (!key) return;
    if (input.value.trim() === '') { input.setAttribute('aria-invalid', 'true'); updateBadges('请输入有效数值'); panel.querySelector('[data-act="apply"]').disabled = true; return; }
    try {
        const r = route(), wp = waypoint();
        const edited = editWaypoint(wp, shotId, key, input.value, locked);
        if (!gesture || gesture !== key) { r.history.push(clone(r.working)); r.history = r.history.slice(-50); gesture = key; }
        r.working[r.working.indexOf(wp)] = edited;
        Scene.updateInspectionWaypoint(currentFile, edited);
        invalidateReports();
        renderSelection();
    } catch (error) { input.setAttribute('aria-invalid', 'true'); updateBadges(error.message); panel.querySelector('[data-act="apply"]').disabled = true; }
}
function invalidateReports() {
    state.lastSafetyResult = null;
    document.getElementById('metrics-panel').textContent = '航点已编辑，原覆盖指标已失效；请在后续评估流程中重新计算。';
    document.getElementById('safety-panel').textContent = '航点已编辑，原安全报告已失效；会话草稿尚未重新校验。';
}
function action(name) {
    if (!name || !route()) return;
    const r = route(); gesture = null;
    if (name === 'close') return closePanel();
    if (name === 'focus') return Scene.flyTo(waypoint());
    if (name === 'prev' || name === 'next') { const list = filtered(), i = list.findIndex(w => w.waypoint_id === selectedId); return select(list[i + (name === 'next' ? 1 : -1)]?.waypoint_id || selectedId); }
    if (name === 'undo') { if (!r.history.length) return; r.working = r.history.pop(); }
    if (name === 'reset') { r.history.push(clone(r.working)); r.working = clone(r.original); r.applied = clone(r.original); }
    if (name === 'apply') r.applied = clone(r.working);
    r.working.forEach(w => Scene.updateInspectionWaypoint(currentFile, w));
    window.dispatchEvent(new CustomEvent('inspection-draft-applied', { detail: { filename: currentFile, waypoints: clone(r.working), dirty: changed(r) } }));
    renderSelection();
    if (name === 'apply') updateBadges('已应用到当前会话。刷新页面会丢失草稿；现有文件下载仍为原始算法结果。');
}
