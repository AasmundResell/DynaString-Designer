import { TelemetryPoint, OperationParams } from "../types";

// Simulates the C++ backend streaming data
export class MockSimulator {
  private intervalId: number | null = null;
  private currentTime = 0;
  private currentDepth = 1000;
  private params: OperationParams;
  private listeners: ((data: TelemetryPoint) => void)[] = [];

  constructor(initialParams: OperationParams) {
    this.params = initialParams;
  }

  public updateParams(newParams: OperationParams) {
    this.params = newParams;
  }

  public subscribe(callback: (data: TelemetryPoint) => void) {
    this.listeners.push(callback);
    return () => {
      this.listeners = this.listeners.filter(cb => cb !== callback);
    };
  }

  public start() {
    if (this.intervalId) return;
    
    this.intervalId = window.setInterval(() => {
      this.currentTime += 0.1;
      
      // Simulate noise and physics response
      const noise = (Math.random() - 0.5);
      
      // WOB fluctuates around target
      const wob = this.params.wob_target + (Math.sin(this.currentTime * 2) * 5) + (noise * 2);
      
      // TOB relates to WOB and RPM
      const tobBase = (wob * 0.1) + (this.params.rpm_target * 0.05);
      const tob = tobBase + (Math.sin(this.currentTime * 5) * 2) + noise;
      
      // Hookload = String weight (constant approx) - WOB + Drag
      const staticWeight = 1500; // kN
      const hookLoad = staticWeight - wob + (Math.cos(this.currentTime) * 10);

      // RPM Surface matches target closely, Bit RPM fluctuates due to stick-slip
      const rpmSurf = this.params.rpm_target + (noise * 0.5);
      
      // Simulate mild stick-slip
      const stickSlipFreq = 0.5;
      const stickSlipAmp = 20;
      const rpmBit = rpmSurf + (Math.sin(this.currentTime * stickSlipFreq) * stickSlipAmp) + noise;
      
      // ROP depends on WOB and RPM
      const rop = (wob * rpmBit * 0.005) + noise;

      // Advance depth
      this.currentDepth += (rop / 3600) * 0.1; // convert m/hr to m/0.1s

      const data: TelemetryPoint = {
        timestamp: this.currentTime,
        depth: this.currentDepth,
        hookLoad: Math.max(0, hookLoad),
        
        wob: Math.max(0, wob),
        tob: Math.max(0, tob),
        rpmSurf: Math.max(0, rpmSurf),
        rpmBit: Math.max(0, rpmBit),
        rop: Math.max(0, rop),
        vibration: Math.abs(Math.sin(this.currentTime * 10)) * 0.5
      };

      this.listeners.forEach(cb => cb(data));

    }, 100); // 10Hz update rate
  }

  public stop() {
    if (this.intervalId) {
      window.clearInterval(this.intervalId);
      this.intervalId = null;
    }
  }
}
