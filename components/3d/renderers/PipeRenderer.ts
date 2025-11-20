import * as THREE from 'three';
import { PipeVertexShader, PipeFragmentShader } from '../shaders/pipeShader';
import { interp_spline_cubic_hermite } from '../math';
import { SimulationConfig, StringComponent } from '../../../types';

export interface PipeRenderComponents {
    polygons: THREE.Vector2[][];
    arr_ie: number[];
    arr_ip: number[];
    arr_S: number[];
}

interface MeshVertex {
    pos: THREE.Vector3;
    normal: THREE.Vector3;
    color: THREE.Vector3;
    uv: THREE.Vector2; // Added UV for texture/pattern if needed
}

export class PipeRenderer {
    public geometry: THREE.BufferGeometry;
    public material: THREE.ShaderMaterial;
    
    private mesh: MeshVertex[] = [];
    private ptr_vertex: number[] = [];
    private ptr_index: number[] = [];
    private index_buffer: number[] = [];
    
    private arr_S_poly: number[] = [];
    private ptr_poly_to_ie: number[] = [];
    
    private NUM_POINTS_CIRCUMF = 20; 

    constructor(
        config: SimulationConfig,
        pipeRenderComponents: PipeRenderComponents,
        pipeAssembly: StringComponent[]
    ) {
        this.arr_S_poly = pipeRenderComponents.arr_S;
        this.ptr_poly_to_ie = pipeRenderComponents.arr_ie;

        this.generateMesh(pipeAssembly, pipeRenderComponents);

        this.geometry = new THREE.BufferGeometry();
        this.updateGeometryBuffers();

        this.material = new THREE.ShaderMaterial({
            vertexShader: PipeVertexShader,
            fragmentShader: PipeFragmentShader,
            uniforms: {
                light_dir: { value: new THREE.Vector3(0, 1, 0) }, 
                view_pos: { value: new THREE.Vector3(0, 0, 0) },
                light_color: { value: new THREE.Vector3(1, 1, 1) },
                ambient_color: { value: new THREE.Vector3(0.3, 0.3, 0.3) },
                specular_strength: { value: 0.5 }
            },
            side: THREE.DoubleSide,
            vertexColors: true
        });
    }

    private generateMesh(
        pipeAssembly: StringComponent[],
        pipeRenderComponents: PipeRenderComponents
    ) {
        const Npoly = pipeRenderComponents.polygons.length;
        this.ptr_index.push(0);
        this.ptr_vertex.push(0);

        for (let i = 0; i < Npoly; i++) {
            const polygon = pipeRenderComponents.polygons[i];
            const S_begin = pipeRenderComponents.arr_S[i];
            const ip = pipeRenderComponents.arr_ip[i];
            
            // ip is the index in the original drillString array
            const comp = pipeAssembly[ip]; 
            // We don't track S_comp_begin exactly here but for visualization S_begin is sufficient
            // as we are placing rings relative to S_begin
            
            this.addPolygonToMesh(S_begin, polygon, ip, comp);
            
            this.ptr_index.push(this.index_buffer.length);
            this.ptr_vertex.push(this.mesh.length);
        }
    }

    private addPolygonToMesh(
        S_begin: number,
        polygon: THREE.Vector2[],
        ip: number,
        comp: StringComponent
    ) {
        const Nseg = polygon.length;
        for (let iseg = 0; iseg < Nseg - 1; iseg++) {
            const { pA, pB, nA, nB } = this.getPolygonPointAndComputeNormals(iseg, polygon);
            this.addPolygonRingToMesh(S_begin, pA, pB, nA, nB, ip, comp);
        }
    }

    private getPolygonPointAndComputeNormals(iseg: number, polygon: THREE.Vector2[]) {
        const Nseg = polygon.length;
        const im = (iseg === 0) ? Nseg - 1 : iseg - 1;
        const i = iseg;
        const ip = (iseg === Nseg - 1) ? 0 : i + 1;
        const ipp = (ip === Nseg - 1) ? 0 : ip + 1;

        const p_im = polygon[im];
        const p_i = polygon[i];
        const p_ip = polygon[ip];
        const p_ipp = polygon[ipp];

        const t_im_seg = new THREE.Vector2().subVectors(p_i, p_im).normalize();
        const t_i_seg = new THREE.Vector2().subVectors(p_ip, p_i).normalize();
        const t_ip_seg = new THREE.Vector2().subVectors(p_ipp, p_ip).normalize();

        const n_im_seg = new THREE.Vector2(t_im_seg.y, -t_im_seg.x);
        const n_i_seg = new THREE.Vector2(t_i_seg.y, -t_i_seg.x);
        const n_ip_seg = new THREE.Vector2(t_ip_seg.y, -t_ip_seg.x);

        const nA = new THREE.Vector2().addVectors(n_i_seg, n_im_seg).normalize();
        const nB = new THREE.Vector2().addVectors(n_ip_seg, n_i_seg).normalize();
        
        return { pA: p_i, pB: p_ip, nA, nB };
    }

    private addPolygonRingToMesh(
        S_begin: number,
        pA: THREE.Vector2,
        pB: THREE.Vector2,
        nA: THREE.Vector2,
        nB: THREE.Vector2,
        ip: number,
        comp: StringComponent
    ) {
        const prev = this.mesh.length;
        
        // Determine color based on component type and radius
        let color = new THREE.Vector3(0.5, 0.5, 0.5); // Default grey
        
        // Check if this part of the polygon is a tool joint or stabilizer
        // pA.y is radius. 
        const radius = pA.y;
        const baseRadius = (comp.od * 0.0254) / 2.0;
        const isLargerDiameter = radius > baseRadius * 1.1; // 10% tolerance

        if (comp.type === 'bit') {
            color = new THREE.Vector3(0.8, 0.5, 0.0); // Orange/Gold for bit
        } else if (comp.stabilizer && isLargerDiameter) {
            color = new THREE.Vector3(0.1, 0.1, 0.1); // Dark for stabilizer blades
        } else if (isLargerDiameter) {
            color = new THREE.Vector3(0.3, 0.3, 0.3); // Dark grey for tool joints
        } else {
            // Body color
             if (comp.type === 'drill_collar') {
                color = new THREE.Vector3(0.4, 0.4, 0.4);
            } else {
                color = new THREE.Vector3(0.7, 0.7, 0.7); // Light grey for pipe body
            }
        }

        for (let i = 0; i < this.NUM_POINTS_CIRCUMF; i++) {
            const theta = (i * 2.0 * Math.PI) / this.NUM_POINTS_CIRCUMF;
            const c = Math.cos(theta);
            const s = Math.sin(theta);

            const vA: MeshVertex = {
                pos: new THREE.Vector3(S_begin + pA.x, c * pA.y, s * pA.y),
                normal: new THREE.Vector3(nA.x, c * nA.y, s * nA.y),
                color: color,
                uv: new THREE.Vector2(i / this.NUM_POINTS_CIRCUMF, 0)
            };

            const vB: MeshVertex = {
                pos: new THREE.Vector3(S_begin + pB.x, c * pB.y, s * pB.y),
                normal: new THREE.Vector3(nB.x, c * nB.y, s * nB.y),
                color: color,
                uv: new THREE.Vector2(i / this.NUM_POINTS_CIRCUMF, 1)
            };

            this.mesh.push(vA);
            this.mesh.push(vB);
        }

        // Indices
        for (let i = 0; i < this.NUM_POINTS_CIRCUMF - 1; i++) {
            this.index_buffer.push(prev + 2 * i);
            this.index_buffer.push(prev + 2 * i + 1);
            this.index_buffer.push(prev + 2 * i + 3);

            this.index_buffer.push(prev + 2 * i);
            this.index_buffer.push(prev + 2 * i + 3);
            this.index_buffer.push(prev + 2 * i + 2);
        }

        // Close segment
        const last = this.NUM_POINTS_CIRCUMF - 1;
        this.index_buffer.push(prev + 2 * last);
        this.index_buffer.push(prev + 2 * last + 1);
        this.index_buffer.push(prev + 1);

        this.index_buffer.push(prev + 2 * last);
        this.index_buffer.push(prev + 1);
        this.index_buffer.push(prev);
    }

    private updateGeometryBuffers() {
        const positions = new Float32Array(this.mesh.length * 3);
        const normals = new Float32Array(this.mesh.length * 3);
        const colors = new Float32Array(this.mesh.length * 3);
        const uvs = new Float32Array(this.mesh.length * 2); // Added UVs

        for (let i = 0; i < this.mesh.length; i++) {
            const v = this.mesh[i];
            positions[i * 3] = v.pos.x;
            positions[i * 3 + 1] = v.pos.y;
            positions[i * 3 + 2] = v.pos.z;

            normals[i * 3] = v.normal.x;
            normals[i * 3 + 1] = v.normal.y;
            normals[i * 3 + 2] = v.normal.z;

            colors[i * 3] = v.color.x;
            colors[i * 3 + 1] = v.color.y;
            colors[i * 3 + 2] = v.color.z;

            uvs[i * 2] = v.uv.x;
            uvs[i * 2 + 1] = v.uv.y;
        }

        this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        this.geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
        this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        this.geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2)); // Set UV attribute
        this.geometry.setIndex(this.index_buffer);
    }

    public update(
        x_pipe: THREE.Vector3[],
        q_pipe: THREE.Quaternion[],
        S_pipe: number[],
        rotationOffset: number = 0
    ) {
        const Npoly = this.ptr_vertex.length - 1;
        const positions = this.geometry.attributes.position.array as Float32Array;
        const normals = this.geometry.attributes.normal.array as Float32Array;

        const ipoly_top = 0;

        // Create a rotation quaternion for the spinning (around local X axis)
        const q_spin = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), rotationOffset);

        for (let ipoly = ipoly_top; ipoly < Npoly; ipoly++) {
            const ie = this.ptr_poly_to_ie[ipoly];
            // Safety check
            if (ie === undefined || ie >= x_pipe.length - 1) continue;

            const S_poly_i = this.arr_S_poly[ipoly];
            const S_poly_ip = this.arr_S_poly[ipoly + 1];

            const S_pipe_i = S_pipe[ie];
            const S_pipe_ip = S_pipe[ie + 1];

            const xi_pipe_i = (S_poly_i - S_pipe_i) / (S_pipe_ip - S_pipe_i);
            const xi_pipe_ip = (S_poly_ip - S_pipe_i) / (S_pipe_ip - S_pipe_i);

            // Interpolate pipe properties
            const scale = S_pipe_ip - S_pipe_i;
            const x_pipe_begin = interp_spline_cubic_hermite(x_pipe[ie], x_pipe[ie+1], q_pipe[ie], q_pipe[ie+1], xi_pipe_i, scale);
            const x_pipe_end = interp_spline_cubic_hermite(x_pipe[ie], x_pipe[ie+1], q_pipe[ie], q_pipe[ie+1], xi_pipe_ip, scale);

            // Slerp quaternions
            const q_start = q_pipe[ie].clone();
            const q_end = q_pipe[ie+1].clone();


            const q_pipe_begin = q_start.clone().slerp(q_end, xi_pipe_i);
            const q_pipe_end = q_start.clone().slerp(q_end, xi_pipe_ip);

            const vert_begin = this.ptr_vertex[ipoly];
            const vert_end = this.ptr_vertex[ipoly + 1];

            for (let ivert = vert_begin; ivert < vert_end; ivert++) {
                const mesh_vertex = this.mesh[ivert];
                const X_vert = mesh_vertex.pos; // Undeformed pos (local)
                const S_poly_new = X_vert.x;

                const xi_poly = (S_poly_new - S_poly_i) / (S_poly_ip - S_poly_i);
                
                const q_xi = q_pipe_begin.clone().slerp(q_pipe_end, xi_poly);
                const x_centre = new THREE.Vector3().lerpVectors(x_pipe_begin, x_pipe_end, xi_poly);

                // Apply spin rotation THEN bending rotation
                // The vertex is defined in a frame where X is axis.
                // Radial vector is (0, y, z).
                // We want to spin it around X first.
                const X_vert_radial = new THREE.Vector3(0, X_vert.y, X_vert.z); 
                X_vert_radial.applyQuaternion(q_spin); // Spin around local X

                // Now transform to global
                const x_vert = x_centre.clone().add(X_vert_radial.applyQuaternion(q_xi));
                
                // Normal transformation
                const normal_current = mesh_vertex.normal.clone();
                normal_current.applyQuaternion(q_spin); // Spin
                normal_current.applyQuaternion(q_xi);   // Bend

                positions[ivert * 3] = x_vert.x;
                positions[ivert * 3 + 1] = x_vert.y;
                positions[ivert * 3 + 2] = x_vert.z;

                normals[ivert * 3] = normal_current.x;
                normals[ivert * 3 + 1] = normal_current.y;
                normals[ivert * 3 + 2] = normal_current.z;
            }
        }

        this.geometry.attributes.position.needsUpdate = true;
        this.geometry.attributes.normal.needsUpdate = true;
    }
}
