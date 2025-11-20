
// Domain Types

export enum ViewMode {
  SETUP = 'SETUP',
  SIMULATION = 'SIMULATION'
}

// --- Configuration Types ---

export interface WellPoint {
  md: number;
  inclination: number;
  azimuth: number;
}

export enum SectionType {
  CASING = 'casing',
  LINER = 'liner',
  OPEN_HOLE = 'open_hole'
}

export interface WellSection {
  id: string;
  name: string;
  type: SectionType;
  md_top: number;
  md_bottom: number;
  od: number; // Outer Diameter of component (e.g. Casing OD)
  id_hole: number; // Inner Diameter (Hole Size / Drift ID)
}

export enum ComponentType {
  DRILL_PIPE = 'drill_pipe',
  HWDP = 'hwdp',
  DRILL_COLLAR = 'drill_collar',
  STABILIZER = 'stabilizer',
  JAR = 'jar',
  SUB = 'sub',
  BIT = 'bit'
}

export interface StabilizerParams {
  bladeOd: number;
  bladeLength: number;
  distFromBottom: number;
}

export interface StringComponent {
  id: string; // internal UI id
  type: ComponentType;
  name: string;
  od: number; // Body Outer Diameter (inch)
  id_pipe: number; // Inner Diameter (inch)
  length: number; // Length (m) per item
  weight: number; // kg/m approx
  count: number; // Number of items in this stack
  stabilizer?: StabilizerParams; // Optional specific settings
}

export interface BitRockParams {
  mu: number; // Friction coefficient
  epsilon: number; // Intrinsic specific energy
  sigma: number; // Uniaxial compressive strength
  gamma: number; // Bit geometric constant
  n_blades: number;
}

export interface ContactParams {
  mu_static: number;
  mu_kinetic: number;
  stribeck_velocity: number;
}

export interface OperationParams {
  rpm_target: number;
  wob_target: number;
  rop_target: number; // m/hr
  flow_rate: number;
  initial_bit_depth: number; // Initial depth of the bit (Feed Depth)
  initial_hole_depth: number; // Initial depth of the drilled hole
}

export interface SimulationConfig {
  wellPath: WellPoint[];
  sections: WellSection[];
  drillString: StringComponent[];
  bitRock: BitRockParams;
  contact: ContactParams;
  operations: OperationParams;
}

// --- Telemetry / Real-time Types ---

export interface TelemetryPoint {
  timestamp: number;
  depth: number; // Bit depth
  hookLoad: number; // kN
  wob: number; // kN
  tob: number; // kNm
  rpmBit: number;
  rpmSurf: number;
  rop: number; // m/hr
  vibration: number; // normalized 0-1
}

export interface TrajectoryPoint3D {
  x: number;
  y: number;
  z: number;
  md: number;
}

// --- C++ Solver Integration Types ---

// Matches ConfigStatic in C++
export interface SolverConfigStatic {
  // Basic Dimensions
  L: number;
  L_string_depth_initial: number;
  
  // Time Integration
  dt: number;
  
  // Physical Properties
  E: number;
  nu: number;
  rho_p: number;
  
  // Fluid
  rho_fi: number;
  rho_fo: number;
  
  // Operation (Initial)
  v_top_input: number;
  omega_top_input: number;
  
  // Bit Rock
  r_br: number;
  m_br: number;
}

// Data streamed from C++ per frame
export interface SolverFrameData {
  time: number;
  depth: number;
  rpm: number;
  // Arrays for spatial plots (matching GUI_Manager extraction)
  points: {
    s: number; // Arc length
    x: number; // Global X
    y: number; // Global Y
    z: number; // Global Z
    qx?: number; // Quaternion X
    qy?: number; // Quaternion Y
    qz?: number; // Quaternion Z
    qw?: number; // Quaternion W
    tension?: number;
  }[];
}

// --- AI Types ---

export interface ChatMessage {
  id: string;
  role: 'user' | 'model' | 'system';
  text: string;
  isToolCall?: boolean;
}
