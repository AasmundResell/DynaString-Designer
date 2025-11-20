
import * as THREE from 'three';
import { SolverFrameData } from '../types';

export const generateDummySolverData = (time: number): SolverFrameData => {
    const points: { s: number; x: number; y: number; z: number; qx: number; qy: number; qz: number; qw: number; }[] = [];
    
    const length = 1000;
    const numPoints = 100;
    const ds = length / numPoints;

    // Create a spiral shape
    for (let i = 0; i <= numPoints; i++) {
        const s = i * ds;
        const t = time * 0.5; // Animation speed
        
        // Spiral parameters
        const radius = 50 + 20 * Math.sin(t * 0.5);
        const pitch = 0.1;
        
        const angle = s * pitch + t;
        
        const x = radius * Math.cos(angle);
        const z = radius * Math.sin(angle);
        const y = -s; // Depth is negative Y in our visualization

        // Calculate tangent for quaternion
        // dx/ds = -r*p*sin(angle)
        // dz/ds = r*p*cos(angle)
        // dy/ds = -1
        const tx = -radius * pitch * Math.sin(angle);
        const tz = radius * pitch * Math.cos(angle);
        const ty = -1;
        
        const tangent = new THREE.Vector3(tx, ty, tz).normalize();
        const up = new THREE.Vector3(0, 0, 1); // Default up
        const quaternion = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), tangent); // Assuming pipe aligns with Y initially? 
        // Wait, in PipeRenderer we assume Z is pipe axis for interpolation? 
        // Let's check math.ts: "Assumes the pipe tangent is along the local Z-axis (0, 0, 1)."
        // So we should align local Z with tangent.
        const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), tangent);

        points.push({
            s,
            x,
            y,
            z,
            qx: q.x,
            qy: q.y,
            qz: q.z,
            qw: q.w
        });
    }

    return {
        time,
        depth: length,
        rpm: 120,
        points
    };
};
