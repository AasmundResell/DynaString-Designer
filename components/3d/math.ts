
import * as THREE from 'three';

/**
 * Interpolates between two points p0 and p1 using Cubic Hermite Spline.
 * The tangents are derived from the quaternions q0 and q1.
 * Assumes the pipe tangent is along the local Z-axis (0, 0, 1).
 * 
 * @param p0 Start position
 * @param p1 End position
 * @param q0 Start rotation
 * @param q1 End rotation
 * @param t Interpolation factor [0, 1]
 * @param scale Tangent scale factor (usually related to segment length)
 */
export function interp_spline_cubic_hermite(
    p0: THREE.Vector3,
    p1: THREE.Vector3,
    q0: THREE.Quaternion,
    q1: THREE.Quaternion,
    t: number,
    scale: number = 1.0
): THREE.Vector3 {
    const t2 = t * t;
    const t3 = t2 * t;

    // Basis functions
    const h00 = 2 * t3 - 3 * t2 + 1;
    const h10 = t3 - 2 * t2 + t;
    const h01 = -2 * t3 + 3 * t2;
    const h11 = t3 - t2;

    // Tangents (assuming Z-axis is the pipe axis)
    const m0 = new THREE.Vector3(0, 0, 1).applyQuaternion(q0).multiplyScalar(scale);
    const m1 = new THREE.Vector3(0, 0, 1).applyQuaternion(q1).multiplyScalar(scale);

    // p(t) = h00*p0 + h10*m0 + h01*p1 + h11*m1
    const result = new THREE.Vector3()
        .addScaledVector(p0, h00)
        .addScaledVector(m0, h10)
        .addScaledVector(p1, h01)
        .addScaledVector(m1, h11);

    return result;
}
