
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
      meshPhysicalMaterial: any;
      planeGeometry: any;
      ambientLight: any;
      pointLight: any;
      directionalLight: any;
      color: any;
      fog: any;
      tubeGeometry: any;
      sphereGeometry: any;
      [elemName: string]: any;
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
const generateDrillStringGeometry = (
  pathPoints: THREE.Vector3[], 
  components: StringComponent[]
) => {
  const curve = new THREE.CatmullRomCurve3(pathPoints);
  const totalPathLength = curve.getLength();
  
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

  const positions: number[] = [];
  const normals: number[] = [];
  const colors: number[] = [];
  const indices: number[] = [];

  const RADIAL_SEGMENTS = 12; 
  let vertexIndex = 0;

  const addRing = (pos: THREE.Vector3, T: THREE.Vector3, N: THREE.Vector3, B: THREE.Vector3, radius: number, color: THREE.Color) => {
    const r = (radius * 0.0254) * 2.0; 

    for (let j = 0; j <= RADIAL_SEGMENTS; j++) {
      const theta = (j / RADIAL_SEGMENTS) * Math.PI * 2;
      const sin = Math.sin(theta);
      const cos = Math.cos(theta);

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

  let currentMD = 0;
  let currentUp = new THREE.Vector3(0, 0, 1); 

  for (const comp of flatString) {
    if (currentMD >= totalPathLength) break;

    type Section = { start: number, end: number, od: number, color: THREE.Color };
    const sections: Section[] = [];
    
    const L = comp.length;
    const OD = comp.od;

    if (comp.type === ComponentType.DRILL_PIPE || comp.type === ComponentType.HWDP) {
      const jointLen = 0.4; 
      const jointOD = OD * 1.4; 
      sections.push({ start: 0, end: jointLen, od: jointOD, color: COLORS.TOOL_JOINT });
      sections.push({ start: jointLen, end: L - jointLen, od: OD, color: COLORS.PIPE_BODY });
      sections.push({ start: L - jointLen, end: L, od: jointOD, color: COLORS.TOOL_JOINT });
    } else if (comp.type === ComponentType.STABILIZER && comp.stabilizer) {
       const sParams = comp.stabilizer;
       const bladeStart = L - sParams.distFromBottom - sParams.bladeLength;
       const bladeEnd = L - sParams.distFromBottom;
       if (bladeStart > 0) sections.push({ start: 0, end: bladeStart, od: OD, color: COLORS.COLLAR });
       sections.push({ start: Math.max(0, bladeStart), end: bladeEnd, od: sParams.bladeOd, color: COLORS.STABILIZER_BLADE });
       if (bladeEnd < L) sections.push({ start: bladeEnd, end: L, od: OD, color: COLORS.COLLAR });
    } else if (comp.type === ComponentType.BIT) {
       sections.push({ start: 0, end: L, od: OD, color: COLORS.BIT });
    } else {
       sections.push({ start: 0, end: L, od: OD, color: COLORS.COLLAR });
    }

    for (const sec of sections) {
      const startGlobal = currentMD + sec.start;
      const endGlobal = currentMD + sec.end;
      
      if (startGlobal >= totalPathLength) continue;

      const stepSize = 2.0; 
      const len = sec.end - sec.start;
      const numSteps = Math.max(1, Math.ceil(len / stepSize));

      for (let i = 0; i <= numSteps; i++) {
         const tLocal = (i / numSteps) * len;
         const md = startGlobal + tLocal;
         if (md > totalPathLength) break;
         
         const u = md / totalPathLength;
         const pos = curve.getPointAt(u);
         const tangent = curve.getTangentAt(u).normalize();
         
         const axis = new THREE.Vector3().crossVectors(currentUp, tangent).normalize();
         if (axis.lengthSq() < 0.001) axis.set(1,0,0); 
         const normal = new THREE.Vector3().crossVectors(tangent, axis).normalize();
         const binormal = axis; 
         
         currentUp.copy(normal); 

         addRing(pos, tangent, normal, binormal, sec.od, sec.color);
         
         if (vertexIndex >= RADIAL_SEGMENTS + 1) {
            const prevRingStart = vertexIndex - (RADIAL_SEGMENTS + 1) * 2;
            const thisRingStart = vertexIndex - (RADIAL_SEGMENTS + 1);
            for (let j = 0; j < RADIAL_SEGMENTS; j++) {
              const a = prevRingStart + j;
              const b = prevRingStart + j + 1;
              const c = thisRingStart + j;
              const d = thisRingStart + j + 1;
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
  geometry.computeVertexNormals();
  return geometry;
};

const DetailedDrillString: React.FC<{ config: SimulationConfig }> = React.memo(({ config }) => {
  const trajectory = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
  const stringGeometry = useMemo(() => {
     return generateDrillStringGeometry(trajectory, config.drillString);
  }, [trajectory, config.drillString]);

  return (
     <mesh geometry={stringGeometry}>
        <meshStandardMaterial vertexColors metalness={0.6} roughness={0.4} />
     </mesh>
  );
});

const WellBore: React.FC<{ config: SimulationConfig, telemetryRef: React.MutableRefObject<TelemetryPoint | undefined> }> = React.memo(({ config, telemetryRef }) => {
   const pathPoints = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
   const fullCurve = useMemo(() => new THREE.CatmullRomCurve3(pathPoints), [pathPoints]);
   const TUBULAR_SEGMENTS = 2000; 
   const RADIAL_SEGMENTS = 20;
   const points = useMemo(() => fullCurve.getSpacedPoints(TUBULAR_SEGMENTS), [fullCurve, TUBULAR_SEGMENTS]); 

   const tubeRef = useRef<any>(null);
   const lineRef = useRef<any>(null);
   
   useEffect(() => {
      if (lineRef.current) {
         lineRef.current.computeLineDistances();
      }
   }, [points]);

   useFrame(() => {
      if (!tubeRef.current || !lineRef.current) return;
      
      const initialHoleDepth = config.operations.initial_hole_depth ?? 1000;
      const currentDepth = telemetryRef.current?.depth || 0;
      const holeDepth = Math.max(initialHoleDepth, currentDepth);
      const totalLen = fullCurve.getLength();
      if (totalLen === 0) return;

      const fraction = Math.min(1, Math.max(0, holeDepth / totalLen));
      const segmentIndex = Math.floor(fraction * TUBULAR_SEGMENTS);
      const indicesPerSegment = RADIAL_SEGMENTS * 6;
      const tubeDrawCount = segmentIndex * indicesPerSegment;

      if (tubeRef.current.geometry) {
         tubeRef.current.geometry.setDrawRange(0, tubeDrawCount);
      }

      if (lineRef.current.geometry) {
         const lineStart = segmentIndex;
         const lineCount = (TUBULAR_SEGMENTS + 1) - lineStart;
         if (lineCount > 0) {
            lineRef.current.geometry.setDrawRange(lineStart, lineCount);
            lineRef.current.visible = true;
         } else {
            lineRef.current.visible = false;
         }
      }
   });

   return (
    <group>
      <mesh ref={tubeRef}>
        <tubeGeometry args={[fullCurve, TUBULAR_SEGMENTS, 20, RADIAL_SEGMENTS, false]} />
        <meshPhysicalMaterial 
          color="#94a3b8" 
          side={THREE.BackSide} 
          transparent 
          opacity={0.3} 
          roughness={0.2}
          metalness={0.5}
          transmission={0.2}
        />
      </mesh>
      
      <line ref={lineRef}>
        <bufferGeometry>
           <bufferAttribute attach="attributes-position" count={points.length} array={new Float32Array(points.flatMap(p => [p.x, p.y, p.z]))} itemSize={3} />
        </bufferGeometry>
        <lineDashedMaterial color="#06b6d4" linewidth={2} opacity={1} transparent dashSize={10} gapSize={5} />
      </line>
    </group>
   );
});

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
    const depth = telemetryRef.current ? telemetryRef.current.depth : (config.operations.initial_bit_depth || 0);
    const rpm = telemetryRef.current ? telemetryRef.current.rpmBit : 0;
    const t = totalLen > 0 ? Math.min(1, Math.max(0, depth / totalLen)) : 0;
    const point = curve.getPointAt(t);
    meshRef.current.position.copy(point);
    const tangent = curve.getTangentAt(t);
    meshRef.current.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), tangent);
    meshRef.current.rotateY(rpm * 0.01);
  });

  return (
    <mesh ref={meshRef}>
       <sphereGeometry args={[4, 16, 16]} />
       <meshStandardMaterial color="#ef4444" emissive="#b91c1c" emissiveIntensity={0.5} transparent opacity={0.8} />
    </mesh>
  );
};

const AxisLabel: React.FC<{ 
  position: [number, number, number], 
  text: string, 
  color?: string, 
  fontSize?: number,
  fontWeight?: string | number,
  anchorY?: "top" | "top-baseline" | "middle" | "bottom-baseline" | "bottom" 
}> = ({ position, text, color = "#334155", fontSize = 60, fontWeight = "bold", anchorY = "middle" }) => {
  return (
    <Billboard position={position}>
      <Text
        color={color}
        fontSize={fontSize}
        fontWeight={fontWeight}
        anchorX="center"
        anchorY={anchorY}
        outlineWidth={fontSize * 0.04}
        outlineColor="#ffffff"
        fillOpacity={1}
      >
        {text}
      </Text>
    </Billboard>
  );
};

const AnnotatedAxes: React.FC<{ config: SimulationConfig }> = React.memo(({ config }) => {
  const path = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
  
  let minY = 0;
  let maxExtent = 0; 

  path.forEach(p => {
      if(p.y < minY) minY = p.y;
      const r = Math.max(Math.abs(p.x), Math.abs(p.z));
      if (r > maxExtent) maxExtent = r;
  });

  const maxDepth = Math.abs(minY);
  const floorY = minY - Math.max(50, maxDepth * 0.05); 

  const padding = Math.max(200, maxExtent * 0.2);
  const limit = maxExtent + padding; 
  const size = limit * 2;

  const depthTicks = useMemo(() => {
      const ticks = [];
      const interval = maxDepth > 5000 ? 1000 : 500;
      for(let d=0; d <= maxDepth; d+=interval) {
          ticks.push(d);
      }
      return ticks;
  }, [maxDepth]);

  const floorTicks = useMemo(() => {
      const ticks = [];
      const step = size > 4000 ? 1000 : (size > 1500 ? 500 : 250);
      const start = Math.ceil(-limit / step) * step;
      const end = Math.floor(limit / step) * step;
      for(let v = start; v <= end; v += step) {
        if (Math.abs(v) > 10 && Math.abs(v) < limit - 10) { 
           ticks.push(v);
        }
      }
      return ticks;
  }, [limit, size]);

  return (
     <group>
        {/* Central TVD Axis Line */}
        <Line points={[[0, 0, 0], [0, floorY, 0]]} color="#64748b" lineWidth={2} />

        {/* TVD Markers */}
        {depthTicks.map(d => (
            <group key={d} position={[0, -d, 0]}>
                <Line points={[[-20, 0, 0], [20, 0, 0]]} color="#64748b" lineWidth={2} />
                <AxisLabel position={[60, 0, 0]} text={`${d}m`} color="#334155" fontSize={60} />
            </group>
        ))}

        {/* Floor Grid & Cardinal Directions */}
        <group position={[0, floorY, 0]}>
            {/* Matte Floor Plane - Made Whiter */}
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.5, 0]} receiveShadow>
                <planeGeometry args={[size, size]} />
                <meshStandardMaterial color="#ffffff" roughness={0.9} metalness={0.0} emissive="#ffffff" emissiveIntensity={0.2} />
            </mesh>

            {/* High Contrast Grid */}
            <Grid 
                position={[0, 0, 0]}
                args={[size, size]} 
                cellSize={100} 
                sectionSize={500} 
                fadeDistance={size * 0.9} 
                sectionColor="#1e293b" 
                cellColor="#64748b"
                infiniteGrid={false}
            />
            
            
            {/* Axes for Orientation - Neutral Colors */}
            {/* North (Z-) Axis */}
            <Line points={[[0, 0, limit], [0, 0, -limit]]} color="#64748b" lineWidth={3} />
            {/* East (X+) Axis */}
            <Line points={[[-limit, 0, 0], [limit, 0, 0]]} color="#64748b" lineWidth={3} />
            
            {/* Cardinal Labels */}
            <AxisLabel position={[0, 20, -limit]} text="North" color="#1e293b" fontSize={60} anchorY="bottom" />
            <AxisLabel position={[0, 20, limit]} text="South" color="#1e293b" fontSize={60} anchorY="bottom" />
            <AxisLabel position={[limit, 20, 0]} text="East" color="#1e293b" fontSize={60} anchorY="bottom" />
            <AxisLabel position={[-limit, 20, 0]} text="West" color="#1e293b" fontSize={60} anchorY="bottom" />

             {/* Numeric Ticks */}
             {floorTicks.map((val, i) => (
                <React.Fragment key={i}>
                    {/* X Axis Labels */}
                    <AxisLabel position={[val, 10, 0]} text={`${Math.abs(val)}m`} fontSize={60} color="#334155" anchorY="bottom" />
                    {/* Z Axis Labels */}
                    <AxisLabel position={[0, 10, val]} text={`${Math.abs(val)}m`} fontSize={60} color="#334155" anchorY="bottom" />
                </React.Fragment>
             ))}
        </group>
     </group>
  );
});

interface SceneProps {
  config: SimulationConfig;
  telemetryRef: React.MutableRefObject<TelemetryPoint | undefined>;
  staticDepth: number;
  currentHoleDepth: number;
}

const Visualization3DComponent: React.FC<SceneProps> = ({ config, telemetryRef }) => {
  const maxDepth = config.wellPath[config.wellPath.length-1].md;
  
  return (
    <div className="w-full h-full bg-slate-50 overflow-hidden border-l border-slate-200 relative shadow-inner">
      <Canvas shadows>
        <PerspectiveCamera makeDefault position={[400, 100, 400]} fov={50} near={1} far={50000} />
        <ambientLight intensity={0.7} />
        <pointLight position={[500, 500, 500]} intensity={0.8} castShadow />
        <directionalLight position={[-500, 1000, 500]} intensity={1.0} castShadow />

        <group>
            <WellBore config={config} telemetryRef={telemetryRef} />
            <DetailedDrillString config={config} />
            <BitMarker config={config} telemetryRef={telemetryRef} />
            <AnnotatedAxes config={config} />
        </group>
        
        <color attach="background" args={['#f8fafc']} />
        <fog attach="fog" args={['#f8fafc', 3000, 50000]} />
        <OrbitControls target={[0, -maxDepth/2, 0]} />
      </Canvas>
      
      <div className="absolute top-4 right-4 z-10 bg-white/80 p-3 rounded border border-slate-200 text-xs text-slate-600 backdrop-blur-sm pointer-events-none select-none">
        <div className="font-bold mb-1">Visualization</div>
        <div>Left Click: Rotate</div>
        <div>Right Click: Pan</div>
        <div>Scroll: Zoom</div>
      </div>
    </div>
  );
};

export const Visualization3D = React.memo(Visualization3DComponent);
