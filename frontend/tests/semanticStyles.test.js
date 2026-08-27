import test from 'node:test';
import assert from 'node:assert/strict';

import {
    LOD_CLASSIFICATION_PALETTE,
    semanticColorOf,
    semanticKindOf,
    SEMANTIC_KIND
} from '../src/legacy/renderers/semanticStyles.js';

test('uses LAS classification labels before category and legacy type', () => {
    assert.equal(
        semanticKindOf({ label: 3, category: 'insulator', type: 3 }),
        SEMANTIC_KIND.GROUND_WIRE
    );
    assert.equal(
        semanticKindOf({ label: 22, category: 'tower', type: 2 }),
        SEMANTIC_KIND.INSULATOR
    );
});

test('falls back to category and legacy type for old voxel payloads', () => {
    assert.equal(semanticKindOf({ category: 'wire' }), SEMANTIC_KIND.CONDUCTOR);
    assert.equal(semanticKindOf({ semantic: 'ground_wire' }), SEMANTIC_KIND.GROUND_WIRE);
    assert.equal(
        semanticKindOf({ category: 'unknown', semantic: 'insulator' }),
        SEMANTIC_KIND.INSULATOR
    );
    assert.equal(semanticKindOf({ type: 2 }), SEMANTIC_KIND.TOWER);
    assert.equal(semanticKindOf({ type: 3 }), SEMANTIC_KIND.INSULATOR);
});

test('shares semantic colors with the Potree classification palette', () => {
    assert.deepEqual(
        LOD_CLASSIFICATION_PALETTE[16].slice(0, 3),
        semanticColorOf({ label: 16 })
    );
    assert.deepEqual(
        LOD_CLASSIFICATION_PALETTE[22].slice(0, 3),
        semanticColorOf({ label: 22 })
    );
    assert.deepEqual(
        LOD_CLASSIFICATION_PALETTE[0].slice(0, 3),
        semanticColorOf({ label: 0 })
    );
    assert.deepEqual(
        LOD_CLASSIFICATION_PALETTE[3].slice(0, 3),
        semanticColorOf({ label: 3 })
    );
});
