export const SEMANTIC_KIND = Object.freeze({
    CONDUCTOR: 'conductor',
    GROUND_WIRE: 'ground_wire',
    TOWER: 'tower',
    INSULATOR: 'insulator',
    OTHER: 'other'
});

export const SEMANTIC_LABEL_KIND = Object.freeze({
    0: SEMANTIC_KIND.CONDUCTOR,
    3: SEMANTIC_KIND.GROUND_WIRE,
    16: SEMANTIC_KIND.TOWER,
    22: SEMANTIC_KIND.INSULATOR
});

export const SEMANTIC_RGB = Object.freeze({
    [SEMANTIC_KIND.CONDUCTOR]: [0.04, 0.50, 0.18],
    [SEMANTIC_KIND.GROUND_WIRE]: [0.40, 1.00, 0.60],
    [SEMANTIC_KIND.TOWER]: [1.00, 0.05, 0.05],
    [SEMANTIC_KIND.INSULATOR]: [0.05, 0.35, 1.00],
    [SEMANTIC_KIND.OTHER]: [0.55, 0.62, 0.74]
});

export const LOD_CLASSIFICATION_PALETTE = Object.freeze({
    DEFAULT: [0.36, 0.40, 0.46, 0.64],
    0: [...SEMANTIC_RGB[SEMANTIC_KIND.CONDUCTOR], 1.0],
    1: [0.42, 0.48, 0.56, 0.86],
    2: [0.48, 0.38, 0.25, 0.92],
    3: [...SEMANTIC_RGB[SEMANTIC_KIND.GROUND_WIRE], 1.0],
    4: [0.28, 0.62, 0.34, 0.92],
    5: [0.18, 0.50, 0.25, 0.92],
    6: [0.55, 0.56, 0.58, 0.88],
    7: [0.78, 0.48, 0.92, 0.95],
    8: [0.10, 0.65, 0.78, 0.95],
    9: [0.20, 0.45, 0.85, 0.95],
    10: [0.76, 0.55, 0.18, 0.95],
    11: [0.72, 0.32, 0.32, 0.95],
    12: [0.32, 0.62, 0.72, 0.95],
    15: [1.0, 0.52, 0.16, 1.0],
    16: [...SEMANTIC_RGB[SEMANTIC_KIND.TOWER], 1.0],
    22: [...SEMANTIC_RGB[SEMANTIC_KIND.INSULATOR], 1.0],
    24: [0.23, 0.51, 0.96, 1.0],
    25: [0.96, 0.62, 0.04, 1.0],
    26: [0.94, 0.27, 0.27, 1.0]
});

const CATEGORY_ALIASES = Object.freeze({
    conductor: SEMANTIC_KIND.CONDUCTOR,
    wire: SEMANTIC_KIND.CONDUCTOR,
    ground_wire: SEMANTIC_KIND.GROUND_WIRE,
    groundwire: SEMANTIC_KIND.GROUND_WIRE,
    tower: SEMANTIC_KIND.TOWER,
    pole: SEMANTIC_KIND.TOWER,
    insulator: SEMANTIC_KIND.INSULATOR
});

export function semanticKindOf(value = {}) {
    const numericLabel = Number(value.label);
    if(Number.isInteger(numericLabel) && SEMANTIC_LABEL_KIND[numericLabel]) {
        return SEMANTIC_LABEL_KIND[numericLabel];
    }

    const semanticFields = [value.category, value.semantic];
    for(const field of semanticFields) {
        const category = `${field || ''}`.trim().toLowerCase();
        if(CATEGORY_ALIASES[category]) return CATEGORY_ALIASES[category];
    }

    const legacyType = Number(value.type);
    if(legacyType === 2) return SEMANTIC_KIND.TOWER;
    if(legacyType === 3) return SEMANTIC_KIND.INSULATOR;
    return SEMANTIC_KIND.OTHER;
}

export function semanticColorOf(value = {}) {
    return SEMANTIC_RGB[semanticKindOf(value)] || SEMANTIC_RGB[SEMANTIC_KIND.OTHER];
}
