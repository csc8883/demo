export const state = {
    user: null, // { name: 'xxx', role: 'admin'/'user' }
    globalOffset: null,
    activeScene: null,
    activeStep: 'data',
    activeSelection: null,
    activeInspectorTab: 'current',
    taskHistory: [],
    operationLog: [],
    lastSafetyResult: null,
    compareVisible: false,
    weightStatuses: {},
    weightProfile: null,
    activeWeightProfileId: null,
    weightCanvasMode: false,
    weightGroups: [],
    weightSelection: {
        interaction: 'navigate',
        tool: 'box3d',
        mode: 'new',
        operations: [],
        selectedGroupId: null
    },
    weightHistory: {
        undo: [],
        redo: []
    },
    weightEditableData: null,
    weightOverlayVisible: false,
    weightDisplayMode: 'classified',
    weightDraftState: '未保存',

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
    state.activeScene = null;
    state.activeStep = 'data';
    state.activeSelection = null;
    state.activeInspectorTab = 'current';
    state.taskHistory = [];
    state.operationLog = [];
    state.lastSafetyResult = null;
    state.compareVisible = false;
    state.weightStatuses = {};
    state.weightProfile = null;
    state.activeWeightProfileId = null;
    state.weightCanvasMode = false;
    state.weightGroups = [];
    state.weightSelection = {
        interaction: 'navigate',
        tool: 'box3d',
        mode: 'new',
        operations: [],
        selectedGroupId: null
    };
    state.weightHistory = { undo: [], redo: [] };
    state.weightEditableData = null;
    state.weightOverlayVisible = false;
    state.weightDisplayMode = 'classified';
    state.weightDraftState = '未保存';
    state.loadedAssets = { pointcloud: [], route: [], voxel: [] };
}
