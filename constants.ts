
import { ComponentType, SectionType, SimulationConfig, StringComponent, WellPoint, WellSection } from "./types";

export const INITIAL_WELL_PATH: WellPoint[] = [
  { md: 0, inclination: 0, azimuth: 0 },
  { md: 500, inclination: 0, azimuth: 0 },
  { md: 1000, inclination: 30, azimuth: 45 },
  { md: 1500, inclination: 60, azimuth: 90 },
  { md: 2000, inclination: 90, azimuth: 90 },
  { md: 3000, inclination: 90, azimuth: 90 },
];

export const INITIAL_SECTIONS: WellSection[] = [
  { id: 'surf-1', name: 'Surface Casing', type: SectionType.CASING, md_top: 0, md_bottom: 500, od: 20, id_hole: 18.5 },
  { id: 'int-1', name: 'Intermediate Casing', type: SectionType.CASING, md_top: 0, md_bottom: 1500, od: 13.375, id_hole: 12.25 },
  { id: 'oh-1', name: 'Open Hole', type: SectionType.OPEN_HOLE, md_top: 1500, md_bottom: 3500, od: 12.25, id_hole: 12.25 },
];

export const AVAILABLE_COMPONENTS: StringComponent[] = [
  { id: 'dp-1', type: ComponentType.DRILL_PIPE, name: "5 7/8'' Drill Pipe", od: 5.875, id_pipe: 5.156, length: 10, weight: 30, count: 1 },
  { id: 'hwdp-1', type: ComponentType.HWDP, name: "5 7/8'' HWDP", od: 5.875, id_pipe: 4.0, length: 9.5, weight: 60, count: 1 },
  { id: 'dc-1', type: ComponentType.DRILL_COLLAR, name: "8 1/4'' Drill Collar", od: 8.25, id_pipe: 3.0, length: 9.1, weight: 150, count: 1 },
  { 
    id: 'stab-1', 
    type: ComponentType.STABILIZER, 
    name: "8 1/4'' Stabilizer", 
    od: 8.25, 
    id_pipe: 3.0, 
    length: 2.0, 
    weight: 180, 
    count: 1,
    stabilizer: { bladeOd: 12.25, bladeLength: 0.8, distFromBottom: 0.6 }
  },
  { id: 'mwd-1', type: ComponentType.SUB, name: "MWD Tool", od: 8.0, id_pipe: 4.0, length: 8.0, weight: 120, count: 1 },
  { id: 'bit-1', type: ComponentType.BIT, name: "PDC Bit 12 1/4''", od: 12.25, id_pipe: 2.0, length: 0.4, weight: 50, count: 1 },
];

export const DEFAULT_CONFIG: SimulationConfig = {
  wellPath: INITIAL_WELL_PATH,
  sections: INITIAL_SECTIONS,
  drillString: [
    { ...AVAILABLE_COMPONENTS[0], count: 30 }, // 30 Drill Pipes
    { ...AVAILABLE_COMPONENTS[1], count: 12 }, // 12 HWDP
    { ...AVAILABLE_COMPONENTS[2], count: 6 },  // 6 Drill Collars
    { ...AVAILABLE_COMPONENTS[4], count: 1 },  // MWD
    { ...AVAILABLE_COMPONENTS[3], count: 1 },  // Stab
    { ...AVAILABLE_COMPONENTS[5], count: 1 },  // Bit
  ],
  bitRock: {
    mu: 0.6,
    epsilon: 50e6,
    sigma: 60e6,
    gamma: 1.0,
    n_blades: 6
  },
  contact: {
    mu_static: 0.4,
    mu_kinetic: 0.3,
    stribeck_velocity: 0.2
  },
  operations: {
    rpm_target: 120,
    wob_target: 50, // kN
    rop_target: 15, // m/hr
    flow_rate: 2000 // l/min
  }
};
