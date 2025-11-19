
import { ComponentType, StringComponent } from "./types";

export type CatalogItem = Omit<StringComponent, 'id' | 'count'>;

// Standard API 5D / RP 7G specifications where applicable
export const COMPONENT_CATALOG: Record<string, CatalogItem[]> = {
  [ComponentType.DRILL_PIPE]: [
    { type: ComponentType.DRILL_PIPE, name: "2-3/8\" Drill Pipe", od: 2.375, id_pipe: 1.815, weight: 6.65, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "2-7/8\" Drill Pipe", od: 2.875, id_pipe: 2.151, weight: 10.40, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "3-1/2\" Drill Pipe", od: 3.500, id_pipe: 2.764, weight: 13.30, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "4\" Drill Pipe", od: 4.000, id_pipe: 3.340, weight: 14.00, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "4-1/2\" Drill Pipe", od: 4.500, id_pipe: 3.826, weight: 16.60, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "5\" Drill Pipe", od: 5.000, id_pipe: 4.276, weight: 19.50, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "5-1/2\" Drill Pipe", od: 5.500, id_pipe: 4.778, weight: 21.90, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "5-7/8\" Drill Pipe", od: 5.875, id_pipe: 5.153, weight: 23.40, length: 9.5 },
    { type: ComponentType.DRILL_PIPE, name: "6-5/8\" Drill Pipe", od: 6.625, id_pipe: 5.965, weight: 25.20, length: 9.5 },
  ],
  [ComponentType.HWDP]: [
    { type: ComponentType.HWDP, name: "3-1/2\" HWDP", od: 3.500, id_pipe: 2.063, weight: 25.3, length: 9.3 },
    { type: ComponentType.HWDP, name: "4\" HWDP", od: 4.000, id_pipe: 2.563, weight: 29.8, length: 9.3 },
    { type: ComponentType.HWDP, name: "4-1/2\" HWDP", od: 4.500, id_pipe: 2.750, weight: 41.0, length: 9.3 },
    { type: ComponentType.HWDP, name: "5\" HWDP", od: 5.000, id_pipe: 3.000, weight: 49.3, length: 9.3 },
    { type: ComponentType.HWDP, name: "5-1/2\" HWDP", od: 5.500, id_pipe: 3.250, weight: 57.0, length: 9.3 },
    { type: ComponentType.HWDP, name: "5-7/8\" HWDP", od: 5.875, id_pipe: 4.000, weight: 60.0, length: 9.3 },
  ],
  [ComponentType.DRILL_COLLAR]: [
    { type: ComponentType.DRILL_COLLAR, name: "3-1/8\" Drill Collar", od: 3.125, id_pipe: 1.250, weight: 22.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "4-1/8\" Drill Collar", od: 4.125, id_pipe: 2.000, weight: 35.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "4-3/4\" Drill Collar", od: 4.750, id_pipe: 2.250, weight: 47.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "6-1/4\" Drill Collar", od: 6.250, id_pipe: 2.250, weight: 91.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "6-1/2\" Drill Collar", od: 6.500, id_pipe: 2.813, weight: 92.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "8\" Drill Collar", od: 8.000, id_pipe: 2.813, weight: 150.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "8-1/4\" Drill Collar", od: 8.250, id_pipe: 2.813, weight: 160.0, length: 9.15 },
    { type: ComponentType.DRILL_COLLAR, name: "9-1/2\" Drill Collar", od: 9.500, id_pipe: 3.000, weight: 217.0, length: 9.15 },
  ],
  [ComponentType.STABILIZER]: [
    { 
      type: ComponentType.STABILIZER, name: "6\" Integral Blade Stab", od: 4.75, id_pipe: 2.25, weight: 60, length: 1.5,
      stabilizer: { bladeOd: 6.0, bladeLength: 0.5, distFromBottom: 0.5 }
    },
    { 
      type: ComponentType.STABILIZER, name: "8-1/2\" Integral Blade Stab", od: 6.5, id_pipe: 2.813, weight: 110, length: 1.8,
      stabilizer: { bladeOd: 8.5, bladeLength: 0.6, distFromBottom: 0.6 }
    },
    { 
      type: ComponentType.STABILIZER, name: "12-1/4\" Integral Blade Stab", od: 8.25, id_pipe: 2.813, weight: 180, length: 2.0,
      stabilizer: { bladeOd: 12.25, bladeLength: 0.8, distFromBottom: 0.6 }
    },
    { 
      type: ComponentType.STABILIZER, name: "17-1/2\" Integral Blade Stab", od: 9.5, id_pipe: 3.0, weight: 240, length: 2.2,
      stabilizer: { bladeOd: 17.5, bladeLength: 1.0, distFromBottom: 0.6 }
    }
  ],
  [ComponentType.BIT]: [
    { type: ComponentType.BIT, name: "6\" PDC Bit", od: 6.0, id_pipe: 1.5, weight: 25, length: 0.25 },
    { type: ComponentType.BIT, name: "8-1/2\" PDC Bit", od: 8.5, id_pipe: 2.0, weight: 45, length: 0.35 },
    { type: ComponentType.BIT, name: "12-1/4\" PDC Bit", od: 12.25, id_pipe: 2.5, weight: 65, length: 0.45 },
    { type: ComponentType.BIT, name: "17-1/2\" PDC Bit", od: 17.5, id_pipe: 3.0, weight: 90, length: 0.55 },
    { type: ComponentType.BIT, name: "6\" Tricone Bit", od: 6.0, id_pipe: 1.5, weight: 30, length: 0.3 },
    { type: ComponentType.BIT, name: "8-1/2\" Tricone Bit", od: 8.5, id_pipe: 2.0, weight: 50, length: 0.4 },
  ],
  [ComponentType.SUB]: [
    { type: ComponentType.SUB, name: "Crossover Sub", od: 6.5, id_pipe: 2.5, weight: 40, length: 1.0 },
    { type: ComponentType.SUB, name: "Bit Sub with Float", od: 6.5, id_pipe: 2.25, weight: 50, length: 1.2 },
    { type: ComponentType.SUB, name: "MWD / LWD Tool", od: 6.75, id_pipe: 3.0, weight: 120, length: 8.0 },
    { type: ComponentType.SUB, name: "Rotary Steerable System (RSS)", od: 6.75, id_pipe: 2.5, weight: 250, length: 6.0 },
    { type: ComponentType.SUB, name: "Mud Motor (PDM)", od: 6.75, id_pipe: 3.0, weight: 300, length: 7.0 },
  ],
  [ComponentType.JAR]: [
    { type: ComponentType.JAR, name: "4-3/4\" Drilling Jar", od: 4.75, id_pipe: 2.25, weight: 70, length: 5.0 },
    { type: ComponentType.JAR, name: "6-1/2\" Drilling Jar", od: 6.5, id_pipe: 2.75, weight: 140, length: 5.5 },
    { type: ComponentType.JAR, name: "8\" Drilling Jar", od: 8.0, id_pipe: 3.0, weight: 220, length: 6.0 },
  ]
};
