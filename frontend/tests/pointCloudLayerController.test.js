import test from 'node:test';
import assert from 'node:assert/strict';

import { PointCloudLayerController } from '../src/legacy/renderers/pointCloudLayerController.js';

class FakeRenderer {
    constructor(type, objectName = type) {
        this.type = type;
        this.object = { name: objectName };
        this.visible = false;
        this.disposed = false;
        this.viewOverride = null;
        this.opacity = null;
        this.pointSize = null;
    }

    setVisible(visible) {
        this.visible = !!visible;
    }

    setViewOverride(override) {
        this.viewOverride = override ? { ...override } : null;
    }

    setOpacity(opacity) {
        this.opacity = opacity;
    }

    setPointSize(pointSize) {
        this.pointSize = pointSize;
    }

    dispose() {
        this.disposed = true;
    }

    getSelectionPreview() {
        return { renderer: this.type };
    }
}

test('atomically swaps the Three fallback for Potree and preserves visibility', () => {
    const changes = [];
    const controller = new PointCloudLayerController({
        id: 'cloud.las',
        onRendererChanged: (next, previous) => changes.push([next?.type || null, previous?.type || null])
    });
    const fallback = new FakeRenderer('three-points');
    const potree = new FakeRenderer('potree-lod');

    controller.setVisible(false);
    assert.equal(controller.commitRenderer(fallback), true);
    assert.equal(fallback.visible, false);

    const request = controller.beginLodRequest({ variant: 'base' });
    assert.equal(controller.commitRenderer(potree, request), true);
    assert.equal(controller.renderer, potree);
    assert.equal(potree.visible, false);
    assert.equal(fallback.visible, false);
    assert.equal(fallback.disposed, true);
    assert.deepEqual(changes, [
        ['three-points', null],
        ['potree-lod', 'three-points']
    ]);
});

test('rejects a late base LOD after an active-weight request becomes current', () => {
    const controller = new PointCloudLayerController({ id: 'cloud.las' });
    const fallback = new FakeRenderer('three-points');
    controller.commitRenderer(fallback);

    const baseRequest = controller.beginLodRequest({ variant: 'base' });
    const weightedRequest = controller.beginLodRequest({
        variant: 'active_weight',
        profileId: 'profile-new'
    });
    const lateBase = new FakeRenderer('potree-lod', 'late-base');
    const weighted = new FakeRenderer('potree-lod', 'weighted');

    assert.equal(controller.commitRenderer(lateBase, baseRequest), false);
    assert.equal(lateBase.disposed, true);
    assert.equal(controller.renderer, fallback);

    assert.equal(controller.commitRenderer(weighted, weightedRequest), true);
    assert.equal(controller.renderer, weighted);
    assert.equal(fallback.disposed, true);
});

test('applies and restores temporary visualization style independently of visibility', () => {
    const controller = new PointCloudLayerController({ id: 'cloud.las' });
    const renderer = new FakeRenderer('potree-lod');
    controller.commitRenderer(renderer);

    controller.setVisible(false);
    controller.setViewOverride({
        colorMode: 'plain',
        opacityFactor: 0.3
    });
    assert.deepEqual(renderer.viewOverride, {
        colorMode: 'plain',
        opacityFactor: 0.3
    });

    const request = controller.beginLodRequest({
        variant: 'active_weight',
        profileId: 'profile-current'
    });
    const replacement = new FakeRenderer('potree-lod', 'weighted-current');
    assert.equal(controller.commitRenderer(replacement, request), true);
    assert.equal(replacement.visible, false);
    assert.deepEqual(replacement.viewOverride, {
        colorMode: 'plain',
        opacityFactor: 0.3
    });

    controller.setViewOverride(null);
    assert.equal(replacement.visible, false);
    assert.equal(replacement.viewOverride, null);
    assert.equal(renderer.disposed, true);
});

test('rejects an older weighted profile when a newer profile request exists', () => {
    const controller = new PointCloudLayerController({ id: 'cloud.las' });
    const oldRequest = controller.beginLodRequest({
        variant: 'active_weight',
        profileId: 'profile-old'
    });
    const newRequest = controller.beginLodRequest({
        variant: 'active_weight',
        profileId: 'profile-new'
    });
    const oldWeighted = new FakeRenderer('potree-lod', 'weighted-old');
    const newWeighted = new FakeRenderer('potree-lod', 'weighted-new');

    assert.equal(controller.commitRenderer(oldWeighted, oldRequest), false);
    assert.equal(oldWeighted.disposed, true);
    assert.equal(controller.commitRenderer(newWeighted, newRequest), true);
    assert.equal(controller.renderer, newWeighted);
});
