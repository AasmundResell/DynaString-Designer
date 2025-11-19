
import React, { useState, useRef, useEffect } from 'react';
import { Send, Bot, User, Sparkles, Loader2, Zap, Play, StopCircle } from 'lucide-react';
import { DrillCopilot } from '../services/ai';
import { ChatMessage, SimulationConfig, StringComponent, TelemetryPoint } from '../types';

interface Props {
  config: SimulationConfig;
  setConfig: React.Dispatch<React.SetStateAction<SimulationConfig>>;
  telemetry: TelemetryPoint[];
  isRunning: boolean;
  onStartSim: () => void;
  onStopSim: () => void;
}

export const AiAssistant: React.FC<Props> = ({ config, setConfig, telemetry, isRunning, onStartSim, onStopSim }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([
    { id: 'init', role: 'model', text: "Hello! I'm your Drilling Copilot. I can help you design your BHA or analyze real-time drilling data. \n\nUse the 'Auto-Optimize' button to let me autonomously iterate on designs." }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOptimizing, setIsOptimizing] = useState(false);
  const [optimizationStatus, setOptimizationStatus] = useState('');

  const copilotRef = useRef<DrillCopilot>(new DrillCopilot());
  const messagesEndRef = useRef<HTMLDivElement>(null);
  
  // Keep a ref to telemetry so the async optimization loop sees fresh data
  const telemetryRef = useRef<TelemetryPoint[]>([]);
  useEffect(() => {
    telemetryRef.current = telemetry;
  }, [telemetry]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleUpdateString = (components: StringComponent[]) => {
    setConfig(prev => ({
      ...prev,
      drillString: components
    }));
  };

  const addMessage = (role: 'user' | 'model', text: string) => {
    setMessages(prev => [...prev, { id: Date.now().toString(), role, text }]);
  };

  const handleSend = async () => {
    if (!input.trim()) return;
    addMessage('user', input);
    setInput('');
    setIsLoading(true);

    try {
      const responseText = await copilotRef.current.sendMessage(
        input,
        config,
        telemetryRef.current,
        handleUpdateString
      );
      addMessage('model', responseText);
    } catch (error) {
      console.error(error);
    } finally {
      setIsLoading(false);
    }
  };

  const runOptimizationLoop = async () => {
    if (isOptimizing) return;
    setIsOptimizing(true);
    const MAX_ITERATIONS = 3;
    const SIM_DURATION_MS = 6000;

    addMessage('model', `Starting Autonomous Optimization Cycle.\nGoal: Maximize ROP, Minimize Vibration.\nMax Iterations: ${MAX_ITERATIONS}`);

    let lastSummary = "No previous run.";

    try {
      for (let i = 0; i < MAX_ITERATIONS; i++) {
        const iterNum = i + 1;
        
        // 1. DESIGN PHASE
        setOptimizationStatus(`Iter ${iterNum}: AI Designing...`);
        const prompt = `[AUTONOMOUS OPTIMIZATION MODE - Iteration ${iterNum}/${MAX_ITERATIONS}]
        
        Current Goal: Maximize ROP (Rate of Penetration) and Minimize Vibration.
        Previous Run Results: ${lastSummary}
        
        INSTRUCTIONS:
        1. Analyze the previous results (if any).
        2. If the results are excellent (ROP > 30 m/hr AND Vibration < 0.1), simply reply "OPTIMAL".
        3. Otherwise, call the 'updateDrillString' tool with a MODIFIED design to improve performance.
        4. Explain your changes briefly.`;

        let designUpdated = false;
        const aiResponse = await copilotRef.current.sendMessage(
          prompt, 
          config, 
          telemetryRef.current, 
          (comps) => {
            handleUpdateString(comps);
            designUpdated = true;
          }
        );

        addMessage('model', `[Iter ${iterNum}] Design Phase:\n${aiResponse}`);

        if (aiResponse.includes("OPTIMAL")) {
          addMessage('model', "Optimization targets reached. Stopping loop.");
          break;
        }

        if (!designUpdated && i > 0) {
           addMessage('model', "No design changes suggested. Stopping optimization.");
           break;
        }

        // 2. TEST PHASE
        setOptimizationStatus(`Iter ${iterNum}: Simulating (5s)...`);
        // Small delay to ensure React state updates config
        await new Promise(r => setTimeout(r, 500)); 
        
        onStartSim();
        
        // Wait for simulation to gather data
        await new Promise(r => setTimeout(r, SIM_DURATION_MS));

        // 3. ANALYZE PHASE
        setOptimizationStatus(`Iter ${iterNum}: Analyzing...`);
        const runData = telemetryRef.current.slice(-50); // Get last ~5 seconds of data
        onStopSim();

        if (runData.length === 0) {
           addMessage('model', "Error: No telemetry captured. Aborting.");
           break;
        }

        const avgRop = (runData.reduce((acc, p) => acc + p.rop, 0) / runData.length).toFixed(2);
        const avgVib = (runData.reduce((acc, p) => acc + p.vibration, 0) / runData.length).toFixed(3);
        const avgWob = (runData.reduce((acc, p) => acc + p.wob, 0) / runData.length).toFixed(1);

        lastSummary = `ROP: ${avgRop} m/hr, Vibration: ${avgVib}, Avg WOB: ${avgWob} kN`;
        addMessage('model', `[Iter ${iterNum}] Test Results:\n${lastSummary}`);

        // Wait a bit before next iteration for UX
        await new Promise(r => setTimeout(r, 1000));
      }
      
      addMessage('model', "Autonomous Optimization Cycle Complete.");

    } catch (e) {
      console.error(e);
      addMessage('model', "An error occurred during the optimization loop.");
    } finally {
      setIsOptimizing(false);
      setOptimizationStatus('');
      onStopSim();
    }
  };

  return (
    <div className="flex flex-col h-full bg-white border-l border-slate-200 shadow-xl relative z-20">
      {/* Header */}
      <div className="p-3 border-b border-slate-100 bg-slate-50 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <div className={`w-8 h-8 rounded-lg flex items-center justify-center shadow-sm transition-colors ${isOptimizing ? 'bg-amber-500' : 'bg-gradient-to-br from-indigo-500 to-purple-600'}`}>
            {isOptimizing ? <Loader2 size={16} className="text-white animate-spin" /> : <Sparkles size={16} className="text-white" />}
          </div>
          <div>
            <h3 className="font-bold text-slate-800 text-sm">DrillBit AI</h3>
            <div className="flex items-center gap-1">
              <span className={`w-2 h-2 rounded-full ${isOptimizing ? 'bg-amber-500' : 'bg-green-500'} animate-pulse`}></span>
              <span className="text-[10px] text-slate-500 font-medium uppercase">
                 {isOptimizing ? optimizationStatus : 'Online'}
              </span>
            </div>
          </div>
        </div>
        
        <button 
          onClick={runOptimizationLoop}
          disabled={isOptimizing || isRunning}
          className={`
             px-3 py-1.5 rounded text-[10px] font-bold uppercase tracking-wider flex items-center gap-1 transition-all
             ${isOptimizing 
               ? 'bg-slate-100 text-slate-400 cursor-not-allowed' 
               : 'bg-white border border-indigo-200 text-indigo-600 hover:bg-indigo-50 hover:border-indigo-300 shadow-sm'}
          `}
        >
           <Zap size={12} fill={isOptimizing ? "none" : "currentColor"} />
           Auto-Optimize
        </button>
      </div>

      {/* Messages */}
      <div className="flex-1 overflow-y-auto p-4 space-y-4 bg-slate-50/50">
        {messages.map((msg) => (
          <div key={msg.id} className={`flex gap-3 ${msg.role === 'user' ? 'flex-row-reverse' : ''}`}>
            <div className={`w-8 h-8 rounded-full flex items-center justify-center shrink-0 ${msg.role === 'user' ? 'bg-slate-200' : 'bg-indigo-100'}`}>
              {msg.role === 'user' ? <User size={16} className="text-slate-600" /> : <Bot size={16} className="text-indigo-600" />}
            </div>
            <div className={`max-w-[85%] p-3 rounded-lg text-sm leading-relaxed shadow-sm ${
              msg.role === 'user' 
                ? 'bg-white text-slate-800 border border-slate-200 rounded-tr-none' 
                : 'bg-indigo-50 text-indigo-950 border border-indigo-100 rounded-tl-none'
            }`}>
              <p className="whitespace-pre-wrap font-sans">{msg.text}</p>
            </div>
          </div>
        ))}
        {isLoading && !isOptimizing && (
          <div className="flex gap-3">
             <div className="w-8 h-8 rounded-full bg-indigo-100 flex items-center justify-center shrink-0">
               <Loader2 size={16} className="text-indigo-600 animate-spin" />
             </div>
             <div className="bg-slate-100 p-3 rounded-lg rounded-tl-none text-xs text-slate-500 italic">
               Processing...
             </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className="p-3 bg-white border-t border-slate-200">
        <div className="relative flex items-center">
          <input 
            type="text" 
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && handleSend()}
            disabled={isOptimizing}
            placeholder={isOptimizing ? "Optimization in progress..." : (isRunning ? "Ask about telemetry..." : "Suggest a drill string...")}
            className="w-full pl-4 pr-10 py-2.5 bg-slate-50 border border-slate-200 rounded-lg focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:border-transparent text-sm text-slate-700 placeholder:text-slate-400 disabled:opacity-50 disabled:cursor-not-allowed"
          />
          <button 
            onClick={handleSend}
            disabled={isLoading || isOptimizing || !input.trim()}
            className="absolute right-1.5 p-1.5 bg-indigo-600 hover:bg-indigo-700 disabled:bg-slate-300 text-white rounded-md transition-colors"
          >
            <Send size={14} />
          </button>
        </div>
      </div>
    </div>
  );
};
