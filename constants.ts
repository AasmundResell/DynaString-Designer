
import { ComponentType, SectionType, SimulationConfig, StringComponent, WellPoint, WellSection } from "./types";
import { COMPONENT_CATALOG } from "./catalog";

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

// Flatten the catalog for legacy usages if necessary, but we generally use the catalog directly now.
export const AVAILABLE_COMPONENTS: StringComponent[] = Object.values(COMPONENT_CATALOG).flat().map(c => ({...c, id: 'temp', count: 1}));

// Helper to quickly get a specific item for default config
const getComp = (type: string, namePartial: string) => {
  const items = COMPONENT_CATALOG[type];
  if (!items) return null;
  return items.find(i => i.name.includes(namePartial));
};

const dp5 = getComp(ComponentType.DRILL_PIPE, "5-7/8") || COMPONENT_CATALOG[ComponentType.DRILL_PIPE][0];
const hwdp5 = getComp(ComponentType.HWDP, "5-7/8") || COMPONENT_CATALOG[ComponentType.HWDP][0];
const dc8 = getComp(ComponentType.DRILL_COLLAR, "8-1/4") || COMPONENT_CATALOG[ComponentType.DRILL_COLLAR][0];
const stab12 = getComp(ComponentType.STABILIZER, "12-1/4") || COMPONENT_CATALOG[ComponentType.STABILIZER][0];
const mwd = getComp(ComponentType.SUB, "MWD") || COMPONENT_CATALOG[ComponentType.SUB][0];
const bit12 = getComp(ComponentType.BIT, "12-1/4") || COMPONENT_CATALOG[ComponentType.BIT][0];

export const DEFAULT_CONFIG: SimulationConfig = {
  wellPath: INITIAL_WELL_PATH,
  sections: INITIAL_SECTIONS,
  drillString: [
    { ...dp5, id: 'dp-1', count: 30 },
    { ...hwdp5, id: 'hwdp-1', count: 12 },
    { ...dc8, id: 'dc-1', count: 6 },
    { ...mwd, id: 'mwd-1', count: 1 },
    { ...stab12, id: 'stab-1', count: 1 },
    { ...bit12, id: 'bit-1', count: 1 },
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
    flow_rate: 2000, // l/min
    initial_bit_depth: 1000, // m
    initial_hole_depth: 1000 // m
  }
};
