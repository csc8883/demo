import test from 'node:test';
import assert from 'node:assert/strict';
import { direction, geometry, optics, localPosition, worldPosition, targetInFrame } from '../src/inspection/cameraModel.js';
import { prepareWaypoints, editWaypoint, componentOf } from '../src/inspection/draftModel.js';
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-8, `${a} != ${b}`);
test('north/east/up camera convention and large world coordinates round trip', () => {
    const origin = [652500, 3423000, 60], p = [652510.25, 3423012.5, 100];
    assert.deepEqual(worldPosition(localPosition(p, origin), origin), p);
    for (const [target, yaw, pitch] of [[[0, 10, 0], 0, 0], [[10, 0, 0], 90, 0], [[0, -10, 0], 180, 0], [[0, 0, -10], 0, -90]]) {
        const g = geometry([0, 0, 0], target); near(g.yaw, yaw); near(g.pitch, pitch);
        direction(g.yaw, g.pitch).forEach((v, i) => near(v * g.Distance, target[i]));
    }
});
test('focal narrowing changes which targets lie in the image', () => {
    assert.ok(optics(84).hfov_deg < optics(24).hfov_deg);
    const s = { yaw: 0, pitch: 0, look_at: [6, 10, 0], f_eq_mm: 24 };
    assert.equal(targetInFrame([0, 0, 0], s), true);
    assert.equal(targetInFrame([0, 0, 0], { ...s, f_eq_mm: 84 }), false);
    assert.equal(targetInFrame([0, 0, 0], { ...s, look_at: [0, -10, 0] }), false);
});
const input = [{ id: 42, position: [0, 10, 10], actionName: 'photo', shots: [
    { shot_id: 'one', look_at: [0, 0, 0], yaw: 180, pitch: -45, f_eq_mm: 35, semantic_focus: 'insulator', instance_id: 7 },
    { shot_id: 'two', look_at: [10, 0, 0], yaw: 135, pitch: -35.26, f_eq_mm: 84, semantic_focus: 'tower_top', target_id: 4 }
] }];
test('move along target ray, update all shots and preserve input/IDs for undo', () => {
    const w = prepareWaypoints(input)[0];
    const next = editWaypoint(w, 'one', 'Distance', 20, true);
    near(geometry(next.pos_utm, next.shots[0].look_at).Distance, 20);
    for (const s of next.shots) { const g = geometry(next.pos_utm, s.look_at); near(g.yaw, s.yaw); near(g.pitch, s.pitch); }
    assert.deepEqual(w.pos_utm, [0, 10, 10]); assert.equal(next.waypoint_id, '42'); assert.equal(next.id, 42);
    assert.equal(componentOf(next.shots[0]).id, 'insulator:instance:7');
});
test('unknown instances are explicit; transit points have no fake shots', () => {
    assert.equal(componentOf({ semantic_focus: 'insulator' }).id, null);
    assert.match(componentOf({ semantic_focus: 'insulator' }).name, /实例未知/);
    assert.equal(prepareWaypoints([{ id: 2, pos_utm: [0, 0, 0], action: 'fly' }])[0].shots.length, 0);
});
test('locked direction rejects editing and invalid values do not enter draft', () => {
    const w = prepareWaypoints(input)[0];
    assert.throws(() => editWaypoint(w, 'one', 'yaw', 40, true));
    assert.throws(() => editWaypoint(w, 'one', 'f_eq_mm', -1, false));
    assert.throws(() => editWaypoint(w, 'one', 'pos_0', NaN, false));
    const next = editWaypoint(w, 'one', 'yaw', 90, false); near(next.shots[0].yaw, 90);
    assert.deepEqual(next.pos_utm, w.pos_utm);
});
