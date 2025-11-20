
import React, { useMemo, useRef, useEffect } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera, Grid, Text, Line, Billboard } from '@react-three/drei';
import * as THREE from 'three';
import { SimulationConfig, WellPoint, TelemetryPoint, StringComponent, ComponentType, SolverFrameData } from '../types';
import { HoleRenderer } from './3d/renderers/HoleRenderer';
import { PipeRenderComponents } from './3d/renderers/PipeRenderer';

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


// --- Components ---

const Pipe3D: React.FC<{ 
    config: SimulationConfig, 
    telemetryRef: React.MutableRefObject<TelemetryPoint | undefined>
}> = ({ config, telemetryRef }) => {
    const meshRef = useRef<THREE.Mesh>(null);
    
    const { geometry, curve } = useMemo(() => {
        // Calculate trajectory points from well path
        const points: THREE.Vector3[] = [new THREE.Vector3(0, 0, 0)];
        let x = 0, y = 0, z = 0;
        
        for (let i = 1; i < config.wellPath.length; i++) {
            const p1 = config.wellPath[i - 1];
            const p2 = config.wellPath[i];
            const dMD = p2.md - p1.md;
            const avgInc = (p1.inclination + p2.inclination) / 2 * (Math.PI / 180);
            const avgAzi = (p1.azimuth + p2.azimuth) / 2 * (Math.PI / 180);

            x += dMD * Math.sin(avgInc) * Math.sin(avgAzi);
            y -= dMD * Math.cos(avgInc);
            z -= dMD * Math.sin(avgInc) * Math.cos(avgAzi);
            
            points.push(new THREE.Vector3(x, y, z));
        }

        const curve = new THREE.CatmullRomCurve3(points);
        const totalLength = curve.getLength();
        
        // Create custom geometry with varying radius for tool joints and stabilizers
        const tubularSegments = 400;
        const radialSegments = 20;
        const positions: number[] = [];
        const normals: number[] = [];
        const indices: number[] = [];
        const colors: number[] = [];
        
        // Flatten drill string components
        interface FlatComponent {
            type: ComponentType;
            od: number;
            length: number;
            stabilizer?: { bladeOd: number; bladeLength: number; distFromBottom: number };
        }
        
        const flatString: FlatComponent[] = [];
        config.drillString.forEach(comp => {
            const count = comp.count || 1;
            for (let i = 0; i < count; i++) {
                flatString.push({
                    type: comp.type,
                    od: comp.od,
                    length: comp.length,
                    stabilizer: comp.stabilizer
                });
            }
        });
        
        let currentMD = 0;
        const componentPositions: { start: number; end: number; comp: FlatComponent }[] = [];
        flatString.forEach(comp => {
            componentPositions.push({
                start: currentMD,
                end: currentMD + comp.length,
                comp
            });
            currentMD += comp.length;
        });
        
        // Generate mesh
        for (let i = 0; i <= tubularSegments; i++) {
            const t = i / tubularSegments;
            const md = t * totalLength;
            const pos = curve.getPointAt(t);
            const tangent = curve.getTangentAt(t);
            
            // Find which component we're in
            let radius = 2.5; // Default radius in meters (converted from inches)
            let color = new THREE.Color(0.6, 0.6, 0.6); // Default grey
            
            for (const { start, end, comp } of componentPositions) {
                if (md >= start && md <= end) {
                    const localMD = md - start;
                    const baseRadius = (comp.od * 0.0254) / 2; // Convert inches to meters
                    
                    // Tool joints for drill pipe and HWDP
                    if (comp.type === ComponentType.DRILL_PIPE || comp.type === ComponentType.HWDP) {
                        const jointLength = 0.4;
                        const jointRadius = baseRadius * 1.5;
                        
                        if (localMD < jointLength) {
                            radius = jointRadius;
                            color = new THREE.Color(0.3, 0.3, 0.3); // Dark grey for joints
                        } else if (localMD > comp.length - jointLength) {
                            radius = jointRadius;
                            color = new THREE.Color(0.3, 0.3, 0.3);
                        } else {
                            radius = baseRadius;
                            color = new THREE.Color(0.7, 0.7, 0.7); // Light grey for body
                        }
                    }
                    // Stabilizer blades
                    else if (comp.type === ComponentType.STABILIZER && comp.stabilizer) {
                        const { bladeOd, bladeLength, distFromBottom } = comp.stabilizer;
                        const bladeStart = comp.length - distFromBottom - bladeLength;
                        const bladeEnd = comp.length - distFromBottom;
                        
                        if (localMD >= bladeStart && localMD <= bladeEnd) {
                            radius = (bladeOd * 0.0254) / 2;
                            color = new THREE.Color(0.1, 0.1, 0.1); // Very dark for blades
                        } else {
                            radius = baseRadius;
                            color = new THREE.Color(0.5, 0.5, 0.5);
                        }
                    }
                    // Drill collars
                    else if (comp.type === ComponentType.DRILL_COLLAR) {
                        radius = baseRadius;
                        color = new THREE.Color(0.4, 0.4, 0.4); // Medium grey
                    }
                    // Bit
                    else if (comp.type === ComponentType.BIT) {
                        radius = baseRadius;
                        color = new THREE.Color(0.8, 0.5, 0.0); // Orange/bronze
                    }
                    else {
                        radius = baseRadius;
                        color = new THREE.Color(0.6, 0.6, 0.6);
                    }
                    break;
                }
            }
            
            // Create normal and binormal
            const normal = new THREE.Vector3();
            const binormal = new THREE.Vector3();
            
            if (Math.abs(tangent.y) < 0.999) {
                normal.set(0, 1, 0).cross(tangent).normalize();
            } else {
                normal.set(1, 0, 0).cross(tangent).normalize();
            }
            binormal.crossVectors(tangent, normal);
            
            // Create ring of vertices
            for (let j = 0; j <= radialSegments; j++) {
                const theta = (j / radialSegments) * Math.PI * 2;
                const sin = Math.sin(theta);
                const cos = Math.cos(theta);
                
                const vx = pos.x + radius * (cos * normal.x + sin * binormal.x);
                const vy = pos.y + radius * (cos * normal.y + sin * binormal.y);
                const vz = pos.z + radius * (cos * normal.z + sin * binormal.z);
                
                const nx = cos * normal.x + sin * binormal.x;
                const ny = cos * normal.y + sin * binormal.y;
                const nz = cos * normal.z + sin * binormal.z;
                
                positions.push(vx, vy, vz);
                normals.push(nx, ny, nz);
                colors.push(color.r, color.g, color.b);
            }
        }
        
        // Generate indices
        for (let i = 0; i < tubularSegments; i++) {
            for (let j = 0; j < radialSegments; j++) {
                const a = i * (radialSegments + 1) + j;
                const b = i * (radialSegments + 1) + j + 1;
                const c = (i + 1) * (radialSegments + 1) + j;
                const d = (i + 1) * (radialSegments + 1) + j + 1;
                
                indices.push(a, c, d);
                indices.push(a, d, b);
            }
        }
        
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geo.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
        geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        geo.setIndex(indices);
        
        return { geometry: geo, curve };
    }, [config.wellPath, config.drillString]);

    // Rotation animation based on RPM
    useFrame(() => {
        if (!meshRef.current || !telemetryRef.current) return;
        
        const rpm = telemetryRef.current.rpmBit || 0;
        const radiansPerSecond = (rpm * 2 * Math.PI) / 60;
        
        // Rotate around the tangent at each point (simplified: rotate around local Y-axis)
        meshRef.current.rotation.y += radiansPerSecond * 0.016; // Approximate frame time
    });

    return (
        <mesh ref={meshRef} geometry={geometry}>
            <meshStandardMaterial 
                vertexColors
                metalness={0.7} 
                roughness={0.3}
            />
        </mesh>
    );
};

const Hole3D: React.FC<{ 
    config: SimulationConfig,
    telemetryRef: React.MutableRefObject<TelemetryPoint | undefined>
}> = ({ config, telemetryRef }) => {
    const { scene } = useThree();
    const rendererRef = useRef<HoleRenderer | null>(null);
    const meshRef = useRef<THREE.Mesh | null>(null);

    useEffect(() => {
        const renderer = new HoleRenderer(config.wellPath);
        rendererRef.current = renderer;

        const mesh = new THREE.Mesh(renderer.geometry, renderer.material);
        mesh.frustumCulled = false;
        scene.add(mesh);
        meshRef.current = mesh;

        return () => {
            if (meshRef.current) {
                scene.remove(meshRef.current);
                meshRef.current.geometry.dispose();
            }
        };
    }, [config.wellPath, scene]);

    useFrame(() => {
        if (!rendererRef.current) return;
        const depth = telemetryRef.current?.depth || 0;
        rendererRef.current.update(depth);
    });

    return null;
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
  // ... (Existing implementation kept for context, simplified for brevity in this replacement)
  // Re-implementing the axes logic to ensure it stays
  const pathPoints = useMemo(() => {
      // Simple trajectory calc for axes bounds
      const points: THREE.Vector3[] = [new THREE.Vector3(0,0,0)];
      let x=0, y=0, z=0;
      for(let i=1; i<config.wellPath.length; i++) {
          const p1 = config.wellPath[i-1];
          const p2 = config.wellPath[i];
          const dMD = p2.md - p1.md;
          const inc = (p1.inclination + p2.inclination)/2 * Math.PI/180;
          const azi = (p1.azimuth + p2.azimuth)/2 * Math.PI/180;
          x += dMD * Math.sin(inc) * Math.sin(azi);
          y -= dMD * Math.cos(inc);
          z -= dMD * Math.sin(inc) * Math.cos(azi);
          points.push(new THREE.Vector3(x,y,z));
      }
      return points;
  }, [config.wellPath]);
  
  let minY = 0;
  let maxExtent = 0; 
  pathPoints.forEach(p => {
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
  solverDataRef?: React.MutableRefObject<SolverFrameData | undefined>; // Made optional for backward compat
  staticDepth: number;
  currentHoleDepth: number;
}

const Visualization3DComponent: React.FC<SceneProps> = ({ config, telemetryRef, solverDataRef }) => {
  const maxDepth = config.wellPath[config.wellPath.length-1].md;
  
  // Fallback ref if not provided
  const internalSolverRef = useRef<SolverFrameData | undefined>(undefined);
  const effectiveSolverRef = solverDataRef || internalSolverRef;

  return (
    <div className="w-full h-full bg-slate-50 overflow-hidden border-l border-slate-200 relative shadow-inner">
      <Canvas shadows>
        <PerspectiveCamera makeDefault position={[400, 100, 400]} fov={50} near={1} far={50000} />
        <ambientLight intensity={0.7} />
        <pointLight position={[500, 500, 500]} intensity={0.8} castShadow />
        <directionalLight position={[-500, 1000, 500]} intensity={1.0} castShadow />

        <group>
            <Hole3D config={config} telemetryRef={telemetryRef} />
            <Pipe3D config={config} telemetryRef={telemetryRef} />
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
        <div className="mt-2 text-blue-600">Using WebGL 2.0 Custom Shaders</div>
      </div>
    </div>
  );
};

export const Visualization3D = React.memo(Visualization3DComponent);
