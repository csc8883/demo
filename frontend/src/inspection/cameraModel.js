// World coordinates are metres, Z-up. Heading: north=0, east=90, clockwise.
export const CAMERA = Object.freeze({ width: 36, height: 24, minFocal: 24, maxFocal: 200 });
const rad = Math.PI / 180;
export function direction(yaw = 0, pitch = 0) {
    return [Math.cos(pitch * rad) * Math.sin(yaw * rad), Math.cos(pitch * rad) * Math.cos(yaw * rad), Math.sin(pitch * rad)];
}
export function geometry(position, target) {
    const d = target.map((v, i) => v - position[i]);
    return { yaw: (Math.atan2(d[0], d[1]) / rad + 360) % 360,
        pitch: Math.atan2(d[2], Math.hypot(d[0], d[1])) / rad, Distance: Math.hypot(...d) };
}
export function optics(focal = 35) {
    const f = Math.max(1, Number(focal) || 35);
    return { hfov_deg: 2 * Math.atan(CAMERA.width / (2 * f)) / rad,
        vfov_deg: 2 * Math.atan(CAMERA.height / (2 * f)) / rad, aspect: CAMERA.width / CAMERA.height };
}
export function localPosition(world, origin) { return world.map((v, i) => v - origin[i]); }
export function worldPosition(local, origin) { return local.map((v, i) => v + origin[i]); }
export function targetInFrame(position, shot) {
    if (!shot.look_at) return null;
    const v = shot.look_at.map((x, i) => x - position[i]);
    const forward = direction(shot.yaw, shot.pitch);
    const right = [Math.cos(shot.yaw * rad), -Math.sin(shot.yaw * rad), 0];
    const up = [-Math.sin(shot.pitch * rad) * Math.sin(shot.yaw * rad), -Math.sin(shot.pitch * rad) * Math.cos(shot.yaw * rad), Math.cos(shot.pitch * rad)];
    const dot = a => a.reduce((s, x, i) => s + x * v[i], 0);
    const z = dot(forward), fov = optics(shot.f_eq_mm);
    return z > 0 && Math.abs(Math.atan2(dot(right), z) / rad) <= fov.hfov_deg / 2
        && Math.abs(Math.atan2(dot(up), z) / rad) <= fov.vfov_deg / 2;
}
