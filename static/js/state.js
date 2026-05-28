export const state = {
    user: null, // { name: 'xxx', role: 'admin'/'user' }
    globalOffset: null,

    // Track loaded assets for the new "Active Layers" feature
    loadedAssets: {
        pointcloud: [], // Array of { id: 'filename', visible: true }
        route: [],      // Array of { id: 'filename', type: 'manual'|'best' }
        voxel: []
    }
};

export function alignCoordinates(rawPos, fileCenter) {
    if(!state.globalOffset) {
        state.globalOffset = fileCenter || rawPos;
        console.log("Global Offset Set:", state.globalOffset);
    }
    return [
        rawPos[0] - state.globalOffset[0],
        rawPos[1] - state.globalOffset[1],
        rawPos[2] - state.globalOffset[2]
    ];
}

export function resetCoordinateCenter() {
    state.globalOffset = null;
}

export function resetState() {
    state.user = null;
    state.globalOffset = null;
    state.loadedAssets = { pointcloud: [], route: [], voxel: [] };
}
