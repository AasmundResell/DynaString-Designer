import React, { useState, useEffect, useRef } from 'react';
import { ViewMode, SimulationConfig, TelemetryPoint } from './types';
import { DEFAULT_CONFIG } from './constants';
import { MockSimulator } from './services/simulator';
import { ConfigurationPanel } from './components/ConfigurationPanel';
import { Visualization3D } from './components/Visualization3D';
import { RealtimeCharts } from './components/RealtimeCharts';
import { Play, Square, Activity, LayoutGrid, Layers, Terminal } from 'lucide-react';

const App: React.FC = () => {
  const [mode, setMode] = useState<ViewMode>(ViewMode.SETUP);
  const [config, setConfig] = useState<SimulationConfig>(DEFAULT_CONFIG);
  const [isRunning, setIsRunning] = useState(false);
  const [telemetryData, setTelemetryData] = useState<TelemetryPoint[]>([]);
  const [currentPoint, setCurrentPoint] = useState<TelemetryPoint | undefined>(undefined);
  
  const simulatorRef = useRef<MockSimulator | null>(null);

  // Initialize simulator instance
  useEffect(() => {
    simulatorRef.current = new MockSimulator(config.operations);
    
    const unsubscribe = simulatorRef.current.subscribe((data) => {
      setCurrentPoint(data);
      setTelemetryData(prev => {
        const newData = [...prev, data];
        if (newData.length > 100) newData.shift(); // Keep buffer size manageable
        return newData;
      });
    });

    return () => {
      unsubscribe();
      simulatorRef.current?.stop();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []); // Run once on mount

  // Update simulator params when config changes (if running)
  useEffect(() => {
    if (simulatorRef.current) {
      simulatorRef.current.updateParams(config.operations);
    }
  }, [config.operations]);

  const toggleSimulation = () => {
    if (isRunning) {
      simulatorRef.current?.stop();
      setIsRunning(false);
      setMode(ViewMode.SETUP);
    } else {
      // Clear old data on restart
      setTelemetryData([]);
      simulatorRef.current?.start();
      setIsRunning(true);
      setMode(ViewMode.SIMULATION);
    }
  };

  return (
    <div className="flex h-screen w-full bg-slate-50 text-slate-900 font-sans overflow-hidden">
      
      {/* SIDEBAR */}
      <div className="w-64 bg-white border-r border-slate-200 flex flex-col shrink-0 shadow-sm z-10">
        <div className="p-6 flex items-center gap-3 border-b border-slate-100">
           <div className="w-8 h-8 bg-gradient-to-br from-cyan-500 to-blue-600 rounded-lg flex items-center justify-center shadow-lg shadow-cyan-500/20">
             <Activity className="text-white" size={20} />
           </div>
           <h1 className="font-bold text-lg tracking-tight text-slate-900">DrillSim Pro</h1>
        </div>

        <div className="flex-1 py-6 px-4 space-y-2">
          <button 
            onClick={() => setMode(ViewMode.SETUP)}
            disabled={isRunning}
            className={`w-full flex items-center gap-3 px-4 py-3 rounded-lg transition-all ${mode === ViewMode.SETUP ? 'bg-slate-100 text-cyan-600 shadow-sm ring-1 ring-slate-200' : 'text-slate-500 hover:bg-slate-50 hover:text-slate-900'} ${isRunning ? 'opacity-50 cursor-not-allowed' : ''}`}
          >
            <LayoutGrid size={20} />
            <span className="font-medium">Configuration</span>
          </button>

          <button 
             onClick={() => setMode(ViewMode.SIMULATION)}
             disabled={!isRunning}
             className={`w-full flex items-center gap-3 px-4 py-3 rounded-lg transition-all ${mode === ViewMode.SIMULATION ? 'bg-slate-100 text-cyan-600 shadow-sm ring-1 ring-slate-200' : 'text-slate-500 hover:bg-slate-50 hover:text-slate-900'} ${!isRunning ? 'opacity-50 cursor-not-allowed' : ''}`}
          >
            <Terminal size={20} />
            <span className="font-medium">Live Telemetry</span>
          </button>
        </div>

        {/* SIMULATION CONTROLS */}
        <div className="p-4 border-t border-slate-100 bg-slate-50">
          <div className="flex items-center justify-between mb-2">
            <span className="text-xs uppercase tracking-wider font-semibold text-slate-500">Status</span>
            <span className={`text-xs font-bold px-2 py-0.5 rounded-full ${isRunning ? 'bg-green-100 text-green-700' : 'bg-slate-200 text-slate-600'}`}>
              {isRunning ? 'RUNNING' : 'IDLE'}
            </span>
          </div>
          
          <button
            onClick={toggleSimulation}
            className={`w-full flex items-center justify-center gap-2 py-3 rounded font-bold text-sm transition-all shadow-md ${isRunning ? 'bg-white text-red-600 border border-red-200 hover:bg-red-50' : 'bg-cyan-600 text-white hover:bg-cyan-500 hover:shadow-cyan-500/20'}`}
          >
            {isRunning ? (
              <> <Square size={16} fill="currentColor" /> STOP SIMULATION </>
            ) : (
              <> <Play size={16} fill="currentColor" /> RUN CASE </>
            )}
          </button>
        </div>
      </div>

      {/* MAIN CONTENT */}
      <div className="flex-1 flex flex-col overflow-hidden bg-slate-50/50">
        
        {/* Header with Key Metrics (Only in Sim Mode) */}
        {mode === ViewMode.SIMULATION && (
           <div className="h-16 bg-white border-b border-slate-200 flex items-center px-6 gap-8 shrink-0 z-20 shadow-sm">
              <div className="flex flex-col">
                 <span className="text-[10px] uppercase text-slate-400 font-bold">Bit Depth</span>
                 <span className="font-mono text-xl text-slate-900">{currentPoint?.depth.toFixed(2)} <span className="text-sm text-slate-400">m</span></span>
              </div>
              <div className="h-8 w-px bg-slate-200"></div>
              <div className="flex flex-col">
                 <span className="text-[10px] uppercase text-slate-400 font-bold">ROP</span>
                 <span className="font-mono text-xl text-emerald-600">{currentPoint?.rop.toFixed(1)} <span className="text-sm text-slate-400">m/hr</span></span>
              </div>
              <div className="h-8 w-px bg-slate-200"></div>
              <div className="flex flex-col">
                 <span className="text-[10px] uppercase text-slate-400 font-bold">WOB</span>
                 <span className="font-mono text-xl text-cyan-600">{currentPoint?.wob.toFixed(1)} <span className="text-sm text-slate-400">kN</span></span>
              </div>
              <div className="h-8 w-px bg-slate-200"></div>
              <div className="flex flex-col">
                 <span className="text-[10px] uppercase text-slate-400 font-bold">Vibration</span>
                 <div className="flex items-center gap-2">
                    <div className="w-20 h-2 bg-slate-200 rounded-full overflow-hidden">
                       <div className="h-full bg-orange-500 transition-all duration-300" style={{ width: `${(currentPoint?.vibration || 0) * 100}%` }}></div>
                    </div>
                 </div>
              </div>
           </div>
        )}

        {/* Workspace */}
        <div className="flex-1 overflow-hidden relative">
           
           {/* Configuration View */}
           {mode === ViewMode.SETUP && (
             <ConfigurationPanel config={config} setConfig={setConfig} />
           )}

           {/* Simulation View */}
           {mode === ViewMode.SIMULATION && (
             <div className="h-full flex flex-col md:flex-row">
                {/* 3D Viewport */}
                <div className="flex-1 relative h-1/2 md:h-full md:w-3/5 p-4">
                   <div className="absolute top-6 left-6 z-10 flex items-center gap-2 text-slate-500/80 font-medium bg-white/50 px-2 py-1 rounded backdrop-blur-sm border border-white/50 shadow-sm">
                      <Layers size={16} /> 3D Visualizer
                   </div>
                   <Visualization3D config={config} telemetry={currentPoint} />
                </div>
                
                {/* Telemetry Charts */}
                <div className="h-1/2 md:h-full md:w-2/5 bg-white border-l border-slate-200 flex flex-col">
                   <div className="p-4 border-b border-slate-100 font-semibold text-slate-800 flex items-center gap-2">
                      <Activity size={16} /> Real-Time Data
                   </div>
                   <div className="flex-1 overflow-hidden">
                      <RealtimeCharts 
                        data={telemetryData} 
                        targetWob={config.operations.wob_target}
                        targetRpm={config.operations.rpm_target}
                      />
                   </div>
                </div>
             </div>
           )}
        </div>
      </div>
    </div>
  );
};

export default App;