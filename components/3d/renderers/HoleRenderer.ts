
import * as THREE from 'three';
import { HoleVertexShader, HoleFragmentShader } from '../shaders/holeShader';
import { WellPoint } from '../../../types';

export class HoleRenderer {
    public geometry: THREE.BufferGeometry;
    public material: THREE.ShaderMaterial;

    private NUM_POINTS_CIRCUMF = 60;
    private meshVertices: number[] = [];
    private meshNormals: number[] = [];
    private meshIndices: number[] = [];

    constructor(wellPath: WellPoint[]) {
        this.geometry = new THREE.BufferGeometry();
        
        // Initial generation based on well path
        this.generateMesh(wellPath);

        this.material = new THREE.ShaderMaterial({
            vertexShader: HoleVertexShader,
            fragmentShader: HoleFragmentShader,
            uniforms: {
                light_dir: { value: new THREE.Vector3(0, 0, -1) },
                view_pos: { value: new THREE.Vector3(0, 0, 0) },
                light_color: { value: new THREE.Vector3(1, 1, 1) },
                object_color: { value: new THREE.Vector3(0.6, 0.6, 0.6) },
                alpha: { value: 0.3 },
                ambient_color: { value: new THREE.Vector3(0.3, 0.3, 0.3) },
                specular_strength: { value: 0.2 }
            },
            side: THREE.BackSide, // Render inside of hole
            transparent: true,
            depthWrite: false // For transparency
        });
    }

    private generateMesh(wellPath: WellPoint[]) {
        // Simplified tube generation along wellpath
        // In a real scenario, we might want to match the C++ "Hole" structure more closely
        // which might involve varying diameters or specific mesh structures.
        // For now, we generate a tube.

        const curvePoints: THREE.Vector3[] = [];
        // Convert wellpoints to 3D points (simplified logic matching Visualization3D.tsx)
        let x = 0, y = 0, z = 0;
        curvePoints.push(new THREE.Vector3(0,0,0));
        
        for (let i = 1; i < wellPath.length; i++) {
            const p1 = wellPath[i-1];
            const p2 = wellPath[i];
            const dMD = p2.md - p1.md;
            const avgInc = (p1.inclination + p2.inclination) / 2 * (Math.PI / 180);
            const avgAzi = (p1.azimuth + p2.azimuth) / 2 * (Math.PI / 180);

            const dZ = dMD * Math.cos(avgInc);
            const dH = dMD * Math.sin(avgInc);
            const dN = dH * Math.cos(avgAzi);
            const dE = dH * Math.sin(avgAzi);

            x += dE;
            y -= dZ;
            z -= dN;
            curvePoints.push(new THREE.Vector3(x, y, z));
        }

        if (curvePoints.length < 2) return;

        const curve = new THREE.CatmullRomCurve3(curvePoints);
        const tubularSegments = 200;
        const radius = 20; // Fixed radius for now

        const frames = curve.computeFrenetFrames(tubularSegments, false);

        for (let i = 0; i <= tubularSegments; i++) {
            const t = i / tubularSegments;
            const pos = curve.getPointAt(t);
            const N = frames.normals[i];
            const B = frames.binormals[i];

            for (let j = 0; j <= this.NUM_POINTS_CIRCUMF; j++) {
                const theta = (j / this.NUM_POINTS_CIRCUMF) * Math.PI * 2;
                const sin = Math.sin(theta);
                const cos = Math.cos(theta);

                const vx = pos.x + radius * (cos * N.x + sin * B.x);
                const vy = pos.y + radius * (cos * N.y + sin * B.y);
                const vz = pos.z + radius * (cos * N.z + sin * B.z);

                const nx = -(cos * N.x + sin * B.x); // Inverted normal for inside view
                const ny = -(cos * N.y + sin * B.y);
                const nz = -(cos * N.z + sin * B.z);

                this.meshVertices.push(vx, vy, vz);
                this.meshNormals.push(nx, ny, nz);
            }
        }

        for (let i = 0; i < tubularSegments; i++) {
            for (let j = 0; j < this.NUM_POINTS_CIRCUMF; j++) {
                const a = i * (this.NUM_POINTS_CIRCUMF + 1) + j;
                const b = i * (this.NUM_POINTS_CIRCUMF + 1) + j + 1;
                const c = (i + 1) * (this.NUM_POINTS_CIRCUMF + 1) + j;
                const d = (i + 1) * (this.NUM_POINTS_CIRCUMF + 1) + j + 1;

                this.meshIndices.push(a, c, d);
                this.meshIndices.push(a, d, b);
            }
        }

        this.geometry.setAttribute('position', new THREE.Float32BufferAttribute(this.meshVertices, 3));
        this.geometry.setAttribute('normal', new THREE.Float32BufferAttribute(this.meshNormals, 3));
        this.geometry.setIndex(this.meshIndices);
    }

    public update(depth: number) {
        // Logic to reveal/hide parts of the hole based on depth
        // This corresponds to setDrawRange in Three.js
        const totalVertices = this.meshVertices.length / 3;
        // Simplified: show everything for now, or implement progressive reveal
        this.geometry.setDrawRange(0, Infinity);
    }
}
