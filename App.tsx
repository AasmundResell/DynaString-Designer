
import React, { useState, useEffect, useRef, useMemo } from 'react';
import { ViewMode, SimulationConfig, TelemetryPoint, SolverFrameData } from './types';
import { DEFAULT_CONFIG } from './constants';
import { solverService } from './services/solverApi';
import { ConfigurationPanel } from './components/ConfigurationPanel';
import { Visualization3D } from './components/Visualization3D';
import { WellSchematic } from './components/WellSchematic';
import { RealtimeCharts } from './components/RealtimeCharts';
import { AiAssistant } from './components/AiAssistant';
import { Play, Square, Activity, Layers, Settings, Box, Map } from 'lucide-react';

const App: React.FC = () => {
  const [config, setConfig] = useState<SimulationConfig>(DEFAULT_CONFIG);
  const [isRunning, setIsRunning] = useState(false);
  const [telemetryData, setTelemetryData] = useState<TelemetryPoint[]>([]);
  const [currentPoint, setCurrentPoint] = useState<TelemetryPoint | undefined>(undefined);
  const [visualMode, setVisualMode] = useState<'3d' | '2d'>('3d');
  const [isAiOpen, setIsAiOpen] = useState(true);
  
  const simulatorRef = useRef<MockSimulator | null>(null);
  const telemetryRef = useRef<TelemetryPoint | undefined>(undefined);

  const maxWellDepth = useMemo(() => {
    if (config.wellPath.length === 0) return 0;
    return config.wellPath[config.wellPath.length - 1].md;
  }, [config.wellPath]);

  const staticDepth = maxWellDepth * 0.5;

  const currentHoleDepth = Math.max(
    config.operations.initial_hole_depth || 0,
    currentPoint?.depth || 0
  );

  useEffect(() => {
    let lastUpdate = 0;

    const handleData = (data: SolverFrameData) => {
      // Map Solver Data to UI Telemetry Format
      const telemetryPoint: TelemetryPoint = {
        timestamp: data.time,
        depth: data.depth,
        rpmBit: data.rpm,
        rpmSurf: data.rpm, // Assuming top drive = bit for now
        wob: 50 + Math.sin(data.time) * 5, // Mock WOB for now (until solver sends it)
        tob: 10 + Math.cos(data.time) * 2, // Mock TOB for now
        hookLoad: 100,
        rop: 20,
        vibration: 0.1
      };

      // Update Ref for 3D view (Immediate - for smooth 60fps animation)
      telemetryRef.current = telemetryPoint;
      
      const now = Date.now();
      if (now - lastUpdate > 100) {
        lastUpdate = now;
        setCurrentPoint(telemetryPoint);
        setTelemetryData(prev => {
          const newData = [...prev, data];
          if (newData.length > 100) newData.shift(); 
          return newData;
        });
      }
    };

    solverService.connectDataStream(handleData);

    return () => {
      solverService.disconnect();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); 

  useEffect(() => {
    if (isRunning) {
      solverService.updateControl({
        v_top_input: config.operations.rop_target / 3600, // Convert m/hr to m/s
        omega_top_input: config.operations.rpm_target * 0.10472 // Convert RPM to rad/s
      }).catch(console.error);
    }
  }, [config.operations, isRunning]);

  const startSimulation = async () => {
    setTelemetryData([]); // Clear old data
    try {
      await solverService.startSimulation({
        // Map initial config to SolverConfigStatic
        L: 1000, // TODO: Calculate from drillString
        L_string_depth_initial: 0,
        dt: 0.001,
        E: 210e9,
        nu: 0.3,
        rho_p: 7850,
        rho_fi: 1000,
        rho_fo: 1000,
        v_top_input: config.operations.rop_target / 3600,
        omega_top_input: config.operations.rpm_target * 0.10472,
        r_br: 0.1,
        m_br: 50
      });
      setIsRunning(true);
    } catch (e) {
      console.error("Failed to start simulation", e);
      alert("Failed to connect to solver. Is the backend running?");
    }
  };

  const stopSimulation = async () => {
    try {
      await solverService.stopSimulation();
      setIsRunning(false);
      setCurrentPoint(undefined);
      telemetryRef.current = undefined;
    } catch (e) {
      console.error("Failed to stop simulation", e);
    }
  };

  const toggleSimulation = () => {
    if (isRunning) {
      stopSimulation();
    } else {
      startSimulation();
    }
  };

  const maxDepth = useMemo(() => Math.max(
    ...config.wellPath.map(p => p.md),
    ...config.sections.map(s => s.md_bottom)
  ), [config]);

  return (
    <div className="flex flex-col h-screen w-full bg-slate-100 text-slate-900 font-sans overflow-hidden">
      
      {/* TOP NAVIGATION BAR */}
      <header className="h-16 bg-slate-900 text-white flex items-center justify-between px-6 shrink-0 shadow-md z-40">
         <div className="flex items-center gap-3">
           <div className="w-9 h-9 bg-gradient-to-br from-cyan-400 to-blue-500 rounded-lg flex items-center justify-center shadow-lg shadow-cyan-500/20">
             <Activity className="text-slate-900" size={22} />
           </div>
           <div>
             <h1 className="font-bold text-lg tracking-tight">DynaString Pro Designer</h1>
             <div className="text-[10px] text-slate-400 font-mono uppercase tracking-widest">Engineering Suite</div>
           </div>
         </div>

         <div className="flex items-center gap-4">
            <div className="flex items-center gap-2 px-4 py-2 bg-slate-800 rounded-lg border border-slate-700">
              <div className={`w-2 h-2 rounded-full ${isRunning ? 'bg-green-500 animate-pulse' : 'bg-slate-500'}`}></div>
              <span className="text-xs font-mono text-slate-300 uppercase">{isRunning ? 'SIMULATION RUNNING' : 'SETUP MODE'}</span>
            </div>

            <button
              onClick={toggleSimulation}
              className={`flex items-center gap-2 px-6 py-2 rounded-lg font-bold text-sm transition-all transform active:scale-95 shadow-lg ${
                isRunning 
                  ? 'bg-red-600 hover:bg-red-700 text-white ring-2 ring-red-500/20' 
                  : 'bg-cyan-500 hover:bg-cyan-400 text-slate-900 ring-2 ring-cyan-400/20'
              }`}
            >
              {isRunning ? <><Square size={16} fill="currentColor" /> STOP</> : <><Play size={16} fill="currentColor" /> SIMULATE</>}
            </button>
         </div>
      </header>

      {/* MAIN CONTENT - SPLIT VIEW */}
      <div className="flex-1 flex overflow-hidden">
        
        {/* LEFT PANEL: Work Area (Config or Charts) */}
        <div className="w-[55%] flex flex-col border-r border-slate-200 bg-slate-50 relative z-20">
           <div className="h-12 border-b border-slate-200 bg-white flex items-center px-4 justify-between shrink-0">
              <h2 className="font-semibold text-slate-700 flex items-center gap-2 text-sm uppercase tracking-wide">
                {isRunning ? (
                  <><Activity size={16} className="text-emerald-600" /> Real-time Telemetry</>
                ) : (
                  <><Settings size={16} className="text-cyan-600" /> Configuration</>
                )}
              </h2>
           </div>

           <div className="flex-1 overflow-hidden relative">
             {isRunning ? (
               <div className="h-full flex flex-col">
                 <div className="p-4 grid grid-cols-3 gap-4 bg-slate-50 border-b border-slate-200">
                    <div className="bg-white p-4 rounded border border-slate-200 shadow-sm">
                       <div className="text-[10px] text-slate-500 font-bold uppercase mb-1">WOB</div>
                       <div className="text-2xl font-mono text-cyan-600">{currentPoint?.wob.toFixed(1) ?? '-'} <span className="text-sm text-slate-400">kN</span></div>
                    </div>
                    <div className="bg-white p-4 rounded border border-slate-200 shadow-sm">
                       <div className="text-[10px] text-slate-500 font-bold uppercase mb-1">RPM (Bit)</div>
                       <div className="text-2xl font-mono text-emerald-600">{currentPoint?.rpmBit.toFixed(0) ?? '-'}</div>
                    </div>
                    <div className="bg-white p-4 rounded border border-slate-200 shadow-sm">
                       <div className="text-[10px] text-slate-500 font-bold uppercase mb-1">Bit Depth</div>
                       <div className="text-2xl font-mono text-slate-800">{currentPoint?.depth.toFixed(2) ?? '0.00'} <span className="text-sm text-slate-400">m</span></div>
                    </div>
                 </div>
                 <div className="flex-1 overflow-hidden">
                    <RealtimeCharts 
                      data={telemetryData} 
                      targetWob={config.operations.wob_target}
                      targetRpm={config.operations.rpm_target}
                    />
                 </div>
               </div>
             ) : (
               <ConfigurationPanel config={config} setConfig={setConfig} />
             )}
           </div>
        </div>

        {/* RIGHT PANEL: 3D & AI */}
        <div className="w-[45%] flex flex-col bg-white border-l border-slate-200">
          
          {/* Visualization (Fills available space) */}
          <div className="flex-1 relative overflow-hidden flex flex-col min-h-0">
             <div className="absolute top-4 left-4 z-10 flex items-center gap-1 bg-white/90 p-1 rounded-md border border-slate-200 shadow-sm">
                <button 
                   onClick={() => setVisualMode('3d')}
                   className={`px-3 py-1.5 rounded flex items-center gap-2 text-xs font-semibold transition-colors ${visualMode === '3d' ? 'bg-slate-800 text-white' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                   <Layers size={14} /> 3D View
                </button>
                <button 
                   onClick={() => setVisualMode('2d')}
                   className={`px-3 py-1.5 rounded flex items-center gap-2 text-xs font-semibold transition-colors ${visualMode === '2d' ? 'bg-slate-800 text-white' : 'text-slate-600 hover:bg-slate-100'}`}
                >
                   <Map size={14} /> 2D Schematic
                </button>
             </div>
             
             <div className="flex-1 relative overflow-hidden">
               {visualMode === '3d' ? (
                  <Visualization3D 
                    config={config} 
                    telemetryRef={telemetryRef}
                    staticDepth={staticDepth}
                    currentHoleDepth={currentHoleDepth}
                  />
               ) : (
                  <WellSchematic 
                    sections={config.sections} 
                    maxDepth={maxDepth} 
                    drillString={config.drillString}
                    rpm={currentPoint?.rpmBit}
                    holeDepth={currentHoleDepth}
                  />
               )}
             </div>
          </div>

          {/* AI Assistant (Collapsible at bottom) */}
          <div className={`transition-all duration-300 ease-in-out flex flex-col border-t border-slate-200 shadow-2xl z-30 ${isAiOpen ? 'h-[45%]' : 'h-14'}`}>
             <AiAssistant 
                config={config} 
                setConfig={setConfig} 
                telemetry={telemetryData} 
                isRunning={isRunning}
                onStartSim={startSimulation}
                onStopSim={stopSimulation}
                isOpen={isAiOpen}
                onToggle={() => setIsAiOpen(!isAiOpen)}
             />
          </div>

        </div>

      </div>
    </div>
  );
};

export default App;
