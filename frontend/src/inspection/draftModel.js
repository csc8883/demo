import { geometry, optics, CAMERA } from './cameraModel.js';
export const clone = value => JSON.parse(JSON.stringify(value));
export const semanticNames = { insulator: '绝缘子', tower_top: '塔头', tower_edge: '塔边缘', tower_body: '塔身', tower_lower30: '塔下部',
    conductor_insulator_connection: '导线挂点', ground_wire_tower_connection: '地线挂点', insulator_tower_side_connection: '横担挂点', tower_base_connection: '塔基', tower: '杆塔', unknown: '未分类' };
export const semanticName = key => semanticNames[key] || key || '未分类';
export function componentOf(shot) {
    const semantic = shot.semantic_focus || shot.AimType || 'unknown';
    const id = shot.component_id || (shot.instance_id != null ? `${semantic}:instance:${shot.instance_id}`
        : shot.target_id != null ? `${semantic}:target:${shot.target_id}` : null);
    return { id, key: id || `unknown:${semantic}`, semantic,
        name: `${semanticName(semantic)} · ${id ? (shot.instance_id != null ? `实例 ${shot.instance_id}` : shot.target_id != null ? `目标 ${shot.target_id}` : id) : '实例未知'}` };
}
export function prepareWaypoints(waypoints) {
    return clone(waypoints).map((wp, i) => {
        wp.waypoint_id = String(wp.waypoint_id ?? wp.id ?? i + 1);
        wp.pos_utm = [...(wp.pos_utm || wp.position)];
        wp.action = wp.action || wp.actionName || (wp.shots?.length ? 'photo' : 'fly');
        wp.point_type ||= wp.action === 'photo' ? 'task' : 'transit';
        wp.shots = (wp.shots?.length ? wp.shots : wp.point_type === 'task' ? [clone(wp)] : []).map((shot, j) => {
            delete shot.shots;
            return { ...shot, shot_id: String(shot.shot_id || `${wp.waypoint_id}:shot:${j + 1}`),
                yaw: Number(shot.yaw ?? shot.heading ?? wp.yaw ?? 0), pitch: Number(shot.pitch ?? wp.pitch ?? 0),
                f_eq_mm: Number(shot.f_eq_mm ?? shot.focal_length_eq_mm ?? wp.f_eq_mm ?? 35),
                look_at: shot.look_at || wp.look_at || null };
        });
        return wp;
    });
}
export function editWaypoint(waypoint, shotId, field, value, locked) {
    const wp = clone(waypoint), shot = wp.shots.find(s => s.shot_id === shotId);
    if (!shot) throw new Error('该航点没有拍摄动作');
    const n = Number(value);
    if (!Number.isFinite(n)) throw new Error('请输入有效数值');
    if (field === 'Distance') {
        if (!locked || !shot.look_at) throw new Error('调整距离需要锁定有效目标');
        if (n < 1 || n > 200) throw new Error('距离范围为 1—200 m');
        const old = geometry(wp.pos_utm, shot.look_at).Distance;
        if (old < 1e-6) throw new Error('航点与目标重合，无法调整距离');
        wp.pos_utm = wp.pos_utm.map((v, i) => shot.look_at[i] + (v - shot.look_at[i]) * n / old);
    } else if (field.startsWith('pos_')) {
        wp.pos_utm[Number(field.slice(4))] = n;
    } else {
        const ranges = { yaw: [0, 360], pitch: [-90, 90], f_eq_mm: [CAMERA.minFocal, CAMERA.maxFocal] };
        if (!ranges[field] || n < ranges[field][0] || n > ranges[field][1]) throw new Error('参数超出允许范围');
        if (locked && ['yaw', 'pitch'].includes(field)) throw new Error('请先解除目标锁定再调整朝向');
        shot[field] = field === 'yaw' ? n % 360 : n;
    }
    // Moving a position changes every shot at that position, not just the selected one.
    for (const s of wp.shots) {
        if (s.look_at) {
            const g = geometry(wp.pos_utm, s.look_at);
            s.Distance = g.Distance;
            if (locked && (field === 'Distance' || field.startsWith('pos_'))) Object.assign(s, g);
        }
        s.heading = s.yaw;
        s.focal_length_eq_mm = s.f_eq_mm;
        Object.assign(s, optics(s.f_eq_mm));
    }
    wp.position = [...wp.pos_utm];
    Object.assign(wp, wp.shots[0]);
    wp.id = waypoint.id;
    return wp;
}
