import React, { useMemo, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera, Grid, Text } from '@react-three/drei';
import * as THREE from 'three';
import { SimulationConfig, WellPoint, TelemetryPoint } from '../types';

// Augment JSX namespace to include R3F intrinsic elements to fix type errors
declare global {
  namespace JSX {
    interface IntrinsicElements {
      mesh: any;
      group: any;
      tubeGeometry: any;
      meshStandardMaterial: any;
      coneGeometry: any;
      arrowHelper: any;
      ambientLight: any;
      pointLight: any;
      directionalLight: any;
      color: any;
    }
  }
}

// --- Helper to calculate 3D coordinates from MD/Inc/Azi ---
const calculateTrajectory = (points: WellPoint[]): THREE.Vector3[] => {
  const path: THREE.Vector3[] = [];
  let x = 0, y = 0, z = 0; // Z is depth in 3D space usually, but Y is Up in ThreeJS.
  // We will map Depth -> -Y. North -> -Z, East -> X.

  // Initial point
  path.push(new THREE.Vector3(0, 0, 0));

  for (let i = 1; i < points.length; i++) {
    const p1 = points[i - 1];
    const p2 = points[i];
    const dMD = p2.md - p1.md;
    
    // Simple Tangential method for visualization purposes
    // Real simulator uses Min Curvature
    const avgInc = (p1.inclination + p2.inclination) / 2 * (Math.PI / 180);
    const avgAzi = (p1.azimuth + p2.azimuth) / 2 * (Math.PI / 180);

    const dZ = dMD * Math.cos(avgInc); // True Vertical Depth delta
    const dH = dMD * Math.sin(avgInc); // Horizontal displacement

    const dN = dH * Math.cos(avgAzi);
    const dE = dH * Math.sin(avgAzi);

    // Update coordinates
    // Map to ThreeJS: x=East, y=-TVD, z=-North
    x += dE;
    y -= dZ; 
    z -= dN;

    path.push(new THREE.Vector3(x, y, z));
  }
  return path;
};

interface WellObjectProps {
  path: THREE.Vector3[];
}

const WellBore: React.FC<WellObjectProps> = ({ path }) => {
  const curve = useMemo(() => new THREE.CatmullRomCurve3(path), [path]);
  
  return (
    <mesh>
      {/* Exaggerated radius (25) for visibility in large world scale */}
      <tubeGeometry args={[curve, 600, 25, 24, false]} />
      <meshStandardMaterial 
        color="#94a3b8" 
        side={THREE.DoubleSide} // Render both sides so it's visible from outside and inside
        transparent 
        opacity={0.25} 
        roughness={0.1}
      />
    </mesh>
  );
};

const DrillString: React.FC<WellObjectProps & { depth: number }> = ({ path, depth }) => {
  const curve = useMemo(() => new THREE.CatmullRomCurve3(path), [path]);
  
  return (
    <mesh>
      {/* Exaggerated radius (8) for visibility */}
      <tubeGeometry args={[curve, 600, 8, 12, false]} />
      <meshStandardMaterial 
        color="#06b6d4" // Cyan
        metalness={0.4} 
        roughness={0.5}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};

interface BitMarkerProps {
  position: THREE.Vector3;
  rotation: number;
}

const BitMarker: React.FC<BitMarkerProps> = ({ position, rotation }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  useFrame(() => {
    if (meshRef.current) {
        meshRef.current.rotation.y += rotation * 0.05;
    }
  });

  return (
    <group position={position}>
       <mesh ref={meshRef} rotation={[Math.PI/2, 0, 0]}>
         {/* Scaled up bit for visibility */}
         <coneGeometry args={[12, 20, 16]} />
         <meshStandardMaterial color="#ef4444" />
       </mesh>
    </group>
  );
}

const CustomAxes = () => {
  return (
    <group position={[0, 1, 0]}>
      {/* N Axis (Z-) */}
      <group>
        <arrowHelper args={[new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, 0), 250, 0x3b82f6, 40, 20]} />
        <Text position={[0, 0, -280]} fontSize={40} color="#3b82f6" anchorX="center" anchorY="middle">N</Text>
      </group>

      {/* E Axis (X+) */}
      <group>
        <arrowHelper args={[new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 250, 0xef4444, 40, 20]} />
        <Text position={[280, 0, 0]} fontSize={40} color="#ef4444" anchorX="center" anchorY="middle">E</Text>
      </group>

      {/* TVD Axis (Y-) */}
      <group>
        <arrowHelper args={[new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 0), 250, 0x22c55e, 40, 20]} />
        <Text position={[40, -280, 0]} fontSize={40} color="#22c55e" anchorX="left" anchorY="middle">TVD</Text>
      </group>
    </group>
  )
}

interface SceneProps {
  config: SimulationConfig;
  telemetry?: TelemetryPoint;
}

export const Visualization3D: React.FC<SceneProps> = ({ config, telemetry }) => {
  const trajectory = useMemo(() => calculateTrajectory(config.wellPath), [config.wellPath]);
  
  // Find bit position based on depth (simplified for demo: place at end)
  const endPoint = trajectory[trajectory.length - 1];

  return (
    <div className="w-full h-full bg-slate-50 rounded-lg overflow-hidden border border-slate-200 relative shadow-inner">
      <div className="absolute top-4 right-4 z-10 bg-white/80 p-3 rounded border border-slate-200 text-xs text-slate-600 backdrop-blur-sm shadow-sm">
        <div className="font-semibold mb-1">Controls</div>
        <div>Left Click: Rotate</div>
        <div>Right Click: Pan</div>
        <div>Scroll: Zoom</div>
      </div>

      <Canvas>
        {/* Camera positioned to see the structure from top-side */}
        <PerspectiveCamera makeDefault position={[400, 200, 400]} fov={45} near={1} far={20000} />
        <OrbitControls target={[0, -500, 0]} maxPolarAngle={Math.PI} enablePan={true} panSpeed={1} zoomSpeed={1} />
        
        {/* Lighting */}
        <ambientLight intensity={0.8} color="#ffffff" />
        <pointLight position={[500, 500, 500]} intensity={1.0} />
        <directionalLight position={[-200, 500, 200]} intensity={1.2} castShadow />

        {/* No Center component to preserve coordinate origin at 0,0,0 */}
        <group>
            <WellBore path={trajectory} />
            <DrillString path={trajectory} depth={telemetry?.depth || 0} />
            <BitMarker position={endPoint} rotation={telemetry?.rpmBit || 0} />
            <CustomAxes />
        </group>
        
        <Grid 
          position={[0, 0, 0]} 
          args={[5000, 5000]} 
          cellSize={250} 
          cellThickness={1} 
          cellColor="#cbd5e1" 
          sectionSize={1000} 
          sectionThickness={1.5} 
          sectionColor="#94a3b8" 
          fadeDistance={8000} 
          infiniteGrid 
        />
        
        <color attach="background" args={['#f8fafc']} />
      </Canvas>
    </div>
  );
};
