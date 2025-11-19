import React, { useMemo, useRef, useEffect } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera, Grid, Text, Line, Billboard } from '@react-three/drei';
import * as THREE from 'three';
import { SimulationConfig, WellPoint, TelemetryPoint, StringComponent, ComponentType } from '../types';

// Fix for missing R3F types in JSX.IntrinsicElements.
declare global {
  namespace JSX {
    interface IntrinsicElements {
      mesh: any;
      group: any;
      bufferGeometry: any;
      bufferAttribute: any;
      meshStandardMaterial: any;
      ambientLight: any;
      pointLight: any;
      directionalLight: any;
      color: any;
      fog: any;
      tubeGeometry: any;
      sphereGeometry: any;
    }
  }
}

declare module 'react' {
  namespace JSX {
    interface IntrinsicElements {
      mesh: any;
      group: any;
      bufferGeometry: any;
      bufferAttribute: any;
      meshStandardMaterial: any;
      ambientLight: any;
      pointLight: any;
      directionalLight: any;
      color: any;
      fog: any;
      tubeGeometry: any;
      sphereGeometry: any;
    }
  }
}

// --- Colors matching Engineering/C++ Style ---
const COLORS = {
  PIPE_BODY: new THREE.Color("#94a3b8"),     // Silver/Grey
  TOOL_JOINT: new THREE.Color("#475569"),    // Darker Grey
  STABILIZER_BLADE: new THREE.Color("#0f172a"), // Almost Black/Dark Blue
  BIT: new THREE.Color("#d97706"),           // Gold/Bronze
  COLLAR: new THREE.Color("#64748b"),        // Drill Collar Grey
};

// --- Trajectory Helpers ---
const calculateTrajectory = (points: WellPoint[]): THREE.Vector3[] => {
  if (!points || points.length === 0) return [new THREE.Vector3(0,0,0)];
  
  const path: THREE.Vector3[] = [];
  let x = 0, y = 0, z = 0; 
  path.push(new THREE.Vector3(0, 0, 0));

  for (let i = 1; i < points.length; i++) {
    const p1 = points[i - 1];
    const p2 = points[i];
    
    if (!p1 || !p2) continue;

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
    
    path.push(new THREE.Vector3(x, y, z));
  }
  return path;
};

// --- Detailed Mesh Generation ---
// Mirrors C++ PipeRenderer: Generates mesh with tool joints, stabilizers, and bit profiles.
const generateDrillStringGeometry = (
  pathPoints: THREE.Vector3[], 
  components: StringComponent[]
) => {
  const curve = new THREE.CatmullRomCurve3(pathPoints);
  const totalPathLength = curve.getLength();
  
  // 1. Flatten Drill String (expand counts)
  // Config is usually Surface -> Bottom.
  interface FlatComp {
    type: ComponentType;
    od: number;
    id: string;
    length: number;
    stabilizer?: any;
  }
  
  const flatString: FlatComp[] = [];
  components.forEach(comp => {
    const count = comp.count || 1;
    for(let i=0; i<count; i++) {
      flatString.push({
        type: comp.type,
        od: comp.od,
        id: comp.id,
        length: comp.length,
        stabilizer: comp.stabilizer
      });
    }
  });

  // 2. Generate Geometry Arrays
  const positions: number[] = [];
  const normals: number[] = [];
  const colors: number[] = [];
  const indices: number[] = [];

  // Simulation Params
  const RADIAL_SEGMENTS = 12; // Low poly for performance
  let vertexIndex = 0;

  // Helper to add a ring
  const addRing = (pos: THREE.Vector3, T: THREE.Vector3, N: THREE.Vector3, B: THREE.Vector3, radius: number, color: THREE.Color) => {
    // Convert inches to generic unit (assuming scale 1 unit = 1 meter, OD is inches)
    // 1 inch = 0.0254 meters. 
    // However, visualization usually exaggerates width. Let's use a factor.
    const r = (radius * 0.0254) * 2.0; // Exaggerate width 2x for visibility

    for (let j = 0; j <= RADIAL_SEGMENTS; j++) {
      const theta = (j / RADIAL_SEGMENTS) * Math.PI * 2;
      const sin = Math.sin(theta);
      const cos = Math.cos(theta);

      // P = Center + r * (cos*N + sin*B)
      const vx = pos.x + r * (cos * N.x + sin * B.x);
      const vy = pos.y + r * (cos * N.y + sin * B.y);
      const vz = pos.z + r * (cos * N.z + sin * B.z);

      const nx = cos * N.x + sin * B.x;
      const ny = cos * N.y + sin * B.y;
      const nz = cos * N.z + sin * B.z;

      positions.push(vx, vy, vz);
      normals.push(nx, ny, nz);
      colors.push(color.r, color.g, color.b);
    }
  };

  // 3. Iterate String and Sample Curve
  let currentMD = 0;
  
  // We need Frenet Frames for the entire curve to prevent twisting
  // We'll sample frames at higher res and interpolate? 
  // Or just compute simple frames on the fly. CatmullRomCurve3 can provide tangents.
  // We will use a parallel transport approx.
  let currentUp = new THREE.Vector3(0, 0, 1); 

  // Loop through components
  for (const comp of flatString) {
    if (currentMD >= totalPathLength) break;

    // Define Profile Sections for this component
    // [RelativeStart, RelativeEnd, OD, Color]
    type Section = { start: number, end: number, od: number, color: THREE.Color };
    const sections: Section[] = [];
    
    const L = comp.length;
    const OD = comp.od;

    if (comp.type === ComponentType.DRILL_PIPE || comp.type === ComponentType.HWDP) {
      // Pipe with Tool Joints
      const jointLen = 0.4; 
      const jointOD = OD * 1.4; // Approx tool joint OD
      
      // Top Joint (Box)
      sections.push({ start: 0, end: jointLen, od: jointOD, color: COLORS.TOOL_JOINT });
      // Body
      sections.push({ start: jointLen, end: L - jointLen, od: OD, color: COLORS.PIPE_BODY });
      // Bottom Joint (Pin)
      sections.push({ start: L - jointLen, end: L, od: jointOD, color: COLORS.TOOL_JOINT });

    } else if (comp.type === ComponentType.STABILIZER && comp.stabilizer) {
       // Stabilizer Profile: Body -> Blade -> Body
       const sParams = comp.stabilizer;
       const bladeStart = L - sParams.distFromBottom - sParams.bladeLength;
       const bladeEnd = L - sParams.distFromBottom;
       
       if (bladeStart > 0) sections.push({ start: 0, end: bladeStart, od: OD, color: COLORS.COLLAR });
       sections.push({ start: Math.max(0, bladeStart), end: bladeEnd, od: sParams.bladeOd, color: COLORS.STABILIZER_BLADE });
       if (bladeEnd < L) sections.push({ start: bladeEnd, end: L, od: OD, color: COLORS.COLLAR });

    } else if (comp.type === ComponentType.BIT) {
       // Bit Profile
       sections.push({ start: 0, end: L, od: OD, color: COLORS.BIT });
    } else {
       // Generic Collar / Sub
       sections.push({ start: 0, end: L, od: OD, color: COLORS.COLLAR });
    }

    // Generate Mesh for Sections
    for (const sec of sections) {
      const startGlobal = currentMD + sec.start;
      const endGlobal = currentMD + sec.end;
      
      if (startGlobal >= totalPathLength) continue;

      // We need to generate rings at start and end of section to form a cylinder
      // To handle sharp transitions (shoulders), we generate a ring at 'start'
      // even if we just generated one at 'end' of previous section (which had diff radius).
      
      const stepSize = 2.0; // meters, sampling resolution inside body
      const len = sec.end - sec.start;
      const numSteps = Math.ceil(len / stepSize);

      for (let i = 0; i <= numSteps; i++) {
         const tLocal = (i / numSteps) * len;
         const md = startGlobal + tLocal;
         if (md > totalPathLength) break;
         
         // Curve sampling
         const u = md / totalPathLength;
         const pos = curve.getPointAt(u);
         const tangent = curve.getTangentAt(u).normalize();
         
         // Compute Frame (Parallel Transportish)
         const axis = new THREE.Vector3().crossVectors(currentUp, tangent).normalize();
         // If tangent is parallel to up, handle it (rare in vert well, but possible)
         if (axis.lengthSq() < 0.001) {
            axis.set(1,0,0); 
         }
         const normal = new THREE.Vector3().crossVectors(tangent, axis).normalize();
         const binormal = axis; // Already normal to tangent
         
         // Store up for next frame to minimize twist
         currentUp.copy(normal); 

         // Add Ring
         addRing(pos, tangent, normal, binormal, sec.od, sec.color);
         
         // Add Indices (connect to previous ring)
         // We generate rings sequentially. 
         // If this is NOT the very first ring of the entire string:
         // However, if we switched sections, we have a "step".
         // The vertexIndex tracks GLOBAL vertices.
         // We need to know if we are continuing a continuous mesh or starting a new detached part?
         // It's one continuous mesh, but radius changes abruptly.
         // Since we generate a ring at `startGlobal` and previous section generated ring at `endGlobal` (same MD),
         // we connect them to form the "Shoulder" face.
         
         if (vertexIndex > RADIAL_SEGMENTS) {
            const prevRingStart = vertexIndex - (RADIAL_SEGMENTS + 1) * 2;
            const thisRingStart = vertexIndex - (RADIAL_SEGMENTS + 1);
            
            for (let j = 0; j < RADIAL_SEGMENTS; j++) {
              const a = prevRingStart + j;
              const b = prevRingStart + j + 1;
              const c = thisRingStart + j;
              const d = thisRingStart + j + 1;
              
              // Faces
              indices.push(a, c, d);
              indices.push(a, d, b);
            }
         }
         
         vertexIndex += (RADIAL_SEGMENTS + 1);
      }
    }

    currentMD += L;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
  geometry.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(normals), 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals(); // Smooth shading
  
  return geometry;
};


// --- Render Components ---

const DetailedDrillString: React.FC<{ config: SimulationConfig }> = React.memo(({ config }) => {
  // Calculate trajectory once
  const trajectory = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
  
  // Generate Heavy Mesh ONLY when config changes
  const stringGeometry = useMemo(() => {
     return generateDrillStringGeometry(trajectory, config.drillString);
  }, [trajectory, config.drillString]);

  return (
     <mesh geometry={stringGeometry}>
        <meshStandardMaterial 
          vertexColors 
          metalness={0.6} 
          roughness={0.4} 
        />
     </mesh>
  );
});

const WellBore: React.FC<{ config: SimulationConfig }> = React.memo(({ config }) => {
   const path = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
   const curve = useMemo(() => new THREE.CatmullRomCurve3(path), [path]);
   
   return (
    <mesh>
      <tubeGeometry args={[curve, 400, 20, 24, false]} />
      <meshStandardMaterial 
        color="#e2e8f0" 
        side={THREE.BackSide} 
        transparent 
        opacity={0.3} 
        roughness={0.8}
      />
    </mesh>
   );
});

// BitMarker tracks the simulation state (depth/rotation) 
// It essentially highlights the "Active" bit position overlaying the static string
const BitMarker: React.FC<{ 
  config: SimulationConfig, 
  telemetryRef: React.MutableRefObject<TelemetryPoint | undefined>,
}> = ({ config, telemetryRef }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  const path = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
  const curve = useMemo(() => new THREE.CatmullRomCurve3(path), [path]);
  const totalLen = curve.getLength();

  useFrame(() => {
    if (!meshRef.current) return;
    const data = telemetryRef.current;
    if (!data) return;

    const depth = data.depth;
    const rpm = data.rpmBit;
    
    // Position
    const t = totalLen > 0 ? Math.min(1, Math.max(0, depth / totalLen)) : 0;
    const point = curve.getPointAt(t);
    meshRef.current.position.copy(point);

    // Orientation
    const tangent = curve.getTangentAt(t);
    meshRef.current.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), tangent);

    // Rotation (Drilling animation)
    meshRef.current.rotateY(rpm * 0.01);
  });

  return (
    <mesh ref={meshRef}>
       {/* A glowing indicator for the active drilling depth */}
       <sphereGeometry args={[4, 16, 16]} />
       <meshStandardMaterial color="#ef4444" emissive="#b91c1c" emissiveIntensity={0.5} transparent opacity={0.8} />
    </mesh>
  );
};

// --- Main Scene ---

const AnnotatedAxes: React.FC<{ config: SimulationConfig }> = React.memo(({ config }) => {
  const path = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
  const box = useMemo(() => new THREE.Box3().setFromPoints(path), [path]);
  const center = useMemo(() => {
     const c = new THREE.Vector3();
     box.getCenter(c);
     return c;
  }, [box]);

  const gridSize = 5000;
  
  return (
     <group position={[0, box.min.y - 50, 0]}>
        <Grid args={[gridSize, gridSize]} cellSize={100} sectionSize={500} fadeDistance={3000} sectionColor="#cbd5e1" cellColor="#e2e8f0" infiniteGrid />
        <Text position={[0, 0, -gridSize/2.5]} fontSize={100} color="#cbd5e1" rotation={[-Math.PI/2, 0, 0]}>NORTH</Text>
     </group>
  );
});

interface SceneProps {
  config: SimulationConfig;
  telemetryRef: React.MutableRefObject<TelemetryPoint | undefined>;
  staticDepth: number;
}

const Visualization3DComponent: React.FC<SceneProps> = ({ config, telemetryRef, staticDepth }) => {
  // Initial Camera Pos based on well depth
  const maxDepth = config.wellPath[config.wellPath.length-1].md;
  
  return (
    <div className="w-full h-full bg-slate-50 overflow-hidden border-l border-slate-200 relative shadow-inner">
      <Canvas shadows>
        <PerspectiveCamera makeDefault position={[400, 100, 400]} fov={50} near={1} far={20000} />
        
        <ambientLight intensity={0.7} />
        <pointLight position={[500, 500, 500]} intensity={0.8} castShadow />
        <directionalLight position={[-500, 1000, 500]} intensity={1.0} castShadow />

        <group>
            <WellBore config={config} />
            <DetailedDrillString config={config} />
            <BitMarker config={config} telemetryRef={telemetryRef} />
            <AnnotatedAxes config={config} />
        </group>
        
        <color attach="background" args={['#f8fafc']} />
        <fog attach="fog" args={['#f8fafc', 1000, 8000]} />
        
        <OrbitControls target={[0, -maxDepth/2, 0]} />
      </Canvas>
      
      <div className="absolute top-4 right-4 z-10 bg-white/80 p-3 rounded border border-slate-200 text-xs text-slate-600 backdrop-blur-sm pointer-events-none">
        <div className="font-bold mb-1">Visualization</div>
        <div>Left Click: Rotate</div>
        <div>Right Click: Pan</div>
        <div>Scroll: Zoom</div>
      </div>
    </div>
  );
};

export const Visualization3D = React.memo(Visualization3DComponent);