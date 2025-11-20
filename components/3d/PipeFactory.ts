import * as THREE from 'three';
import { SimulationConfig, StringComponent, ComponentType } from '../../types';
import { PipeRenderComponents } from './renderers/PipeRenderer';

enum ProfileShape {
    FLAT,        // Regular pipe section or middle of component
    L_LEFT,      // Start of component with tool joint
    L_RIGHT,     // End of component with tool joint
    DOUBLE_STEP, // Short component with tool joints at both ends
    MIDDLE_STEP  // For stabilizers
}

interface ProfileDimensions {
    L1: number; L2: number; L3: number; L4: number;
    D1: number; D2: number; D3: number; D4: number;
}

const SMALL_SCALAR = 1e-6;

function classifyProfileShapeAndDimensions(
    component: StringComponent,
    S_splits: number[],
    k: number,
    dims: ProfileDimensions
): ProfileShape {
    const start_pos = (k === 0) ? 0 : S_splits[k - 1];
    const end_pos = S_splits[k];
    const Lc = end_pos - start_pos;

    // Reset dimensions
    dims.L1 = dims.L2 = dims.L3 = dims.L4 = 0;
    dims.D1 = dims.D2 = dims.D3 = dims.D4 = 0;

    // Convert inches to meters for diameters
    const Do = component.od * 0.0254;
    const Di = component.id * 0.0254;
    
    // Tool joint dimensions (defaults if not specified)
    // Assuming tool joint length is 0.4m and diameter is 1.5x OD if not specified
    const L_tool = 0.4; 
    const D_tool = Do * 1.5;

    const has_tool_joints = component.type === ComponentType.DRILL_PIPE || component.type === ComponentType.HWDP;
    const has_stabilizer = component.type === ComponentType.STABILIZER && !!component.stabilizer;

    if (has_tool_joints) {
        const has_start_tool = start_pos < L_tool;
        const has_end_tool = end_pos > (component.length - L_tool);

        if (has_start_tool && has_end_tool) {
            // DOUBLE_STEP
            dims.L1 = L_tool;
            dims.L2 = component.length - L_tool;
            dims.L3 = component.length;
            dims.D1 = D_tool;
            dims.D2 = Do;
            dims.D3 = D_tool;
            return ProfileShape.DOUBLE_STEP;
        }
        if (has_start_tool) {
            // L_LEFT
            dims.L1 = L_tool;
            dims.L2 = Lc;
            dims.D1 = D_tool;
            dims.D2 = Do;
            return ProfileShape.L_LEFT;
        }
        if (has_end_tool) {
            // L_RIGHT
            dims.L1 = Lc - L_tool;
            dims.L2 = Lc;
            dims.D1 = Do;
            dims.D2 = D_tool;
            return ProfileShape.L_RIGHT;
        }
    } else if (has_stabilizer && component.stabilizer) {
        const { bladeOd, bladeLength, distFromBottom } = component.stabilizer;
        const D_stab = bladeOd * 0.0254;
        const L_stab = bladeLength;
        const S_stab = component.length - distFromBottom - L_stab / 2; // Center of stabilizer

        const stab_start = S_stab - 0.5 * L_stab;
        const stab_end = S_stab + 0.5 * L_stab;

        // Fully contains stabilizer
        if (start_pos <= stab_start + SMALL_SCALAR && end_pos >= stab_end - SMALL_SCALAR) {
            // MIDDLE_STEP (stabilizer fully inside polygon)
            dims.L1 = stab_start - start_pos;                          // Length before stabilizer
            dims.L2 = stab_start - start_pos + L_stab;                 // End of stabilizer relative to start_pos
            dims.L3 = Lc;                                              // End of polygon
            dims.D1 = Do;
            dims.D2 = D_stab;
            dims.D3 = Do;
            return ProfileShape.MIDDLE_STEP;
        }
        // Left split on stabilizer
        if (start_pos < stab_start && end_pos > stab_start && end_pos <= stab_end) {
            dims.L1 = stab_start - start_pos;
            dims.L2 = Lc;
            dims.D1 = Do;
            dims.D2 = D_stab;
            return ProfileShape.L_LEFT;
        }
        // Right split on stabilizer
        if (start_pos >= stab_start && start_pos < stab_end && end_pos > stab_end) {
            dims.L1 = stab_end - start_pos;
            dims.L2 = Lc;
            dims.D1 = D_stab;
            dims.D2 = Do;
            return ProfileShape.L_RIGHT;
        }
        // Polygon is entirely within stabilizer
        if (start_pos >= stab_start && end_pos <= stab_end) {
            dims.L1 = Lc;
            dims.D1 = D_stab;
            return ProfileShape.FLAT;
        }
    }

    // Default: flat
    dims.L1 = Lc;
    dims.D1 = Do;
    return ProfileShape.FLAT;
}

function createPolygon(component: StringComponent, S_splits: number[], k: number): THREE.Vector2[] {
    const dims: ProfileDimensions = { L1: 0, L2: 0, L3: 0, L4: 0, D1: 0, D2: 0, D3: 0, D4: 0 };
    const shape = classifyProfileShapeAndDimensions(component, S_splits, k, dims);
    const Di = component.id * 0.0254;

    const poly: THREE.Vector2[] = [];
    poly.push(new THREE.Vector2(0.0, Di / 2.0));

    switch (shape) {
        case ProfileShape.FLAT:
            poly.push(new THREE.Vector2(0.0, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, Di / 2.0));
            break;
        case ProfileShape.L_LEFT:
            poly.push(new THREE.Vector2(0.0, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, Di / 2.0));
            break;
        case ProfileShape.L_RIGHT:
            poly.push(new THREE.Vector2(0.0, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, Di / 2.0));
            break;
        case ProfileShape.DOUBLE_STEP:
            poly.push(new THREE.Vector2(0.0, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, dims.D3 / 2.0));
            poly.push(new THREE.Vector2(dims.L3, dims.D3 / 2.0));
            poly.push(new THREE.Vector2(dims.L3, Di / 2.0));
            break;
        case ProfileShape.MIDDLE_STEP:
            poly.push(new THREE.Vector2(0.0, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D1 / 2.0));
            poly.push(new THREE.Vector2(dims.L1, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, dims.D2 / 2.0));
            poly.push(new THREE.Vector2(dims.L2, dims.D3 / 2.0));
            poly.push(new THREE.Vector2(dims.L3, dims.D3 / 2.0));
            poly.push(new THREE.Vector2(dims.L3, Di / 2.0));
            break;
        default:
            console.error("Unknown profile shape");
    }

    // Reverse order for correct winding (C++ does this)
    return poly.reverse();
}

export function createPipeRenderComponents(config: SimulationConfig): PipeRenderComponents {
    const dS_max_target = 10.0; // Render segment length

    const polygons: THREE.Vector2[][] = [];
    const arr_ie: number[] = [];
    const arr_ip: number[] = [];
    const arr_S: number[] = [];

    let global_S = 0.0;
    arr_S.push(global_S);
    
    // Flatten drill string components first
    const flatComponents: { comp: StringComponent; index: number }[] = [];
    config.drillString.forEach((comp, index) => {
        const count = comp.count || 1;
        for (let i = 0; i < count; i++) {
            flatComponents.push({ comp, index });
        }
    });

    flatComponents.forEach(({ comp, index }) => {
        let component_pos = 0.0;
        let remaining_length = comp.length;
        const S_splits: number[] = [];

        // Simplified splitting logic: just split by dS_max_target
        // In C++ it maps to FEM elements, here we assume uniform or just visual splitting
        const num_splits = Math.max(1, Math.ceil(comp.length / dS_max_target));
        const sub_length = comp.length / num_splits;

        for (let i = 0; i < num_splits; i++) {
            S_splits.push((i + 1) * sub_length);
        }

        // Create render components based on splits
        for (let k = 0; k < S_splits.length; k++) {
            arr_S.push(global_S + S_splits[k]);
            arr_ip.push(index);
            polygons.push(createPolygon(comp, S_splits, k));
        }

        global_S += comp.length;
    });

    // Generate arr_ie (element indices) - simplified 1-to-1 for now
    // In a real solver connection, we'd map S to solver elements
    for (let i = 0; i < arr_S.length - 1; i++) {
        arr_ie.push(i); 
    }

    return { polygons, arr_ie, arr_ip, arr_S };
}
