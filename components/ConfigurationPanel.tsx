import React, { useState, useMemo } from 'react';
import { SimulationConfig, WellPoint, StringComponent, ComponentType, SectionType, WellSection } from '../types';
import { AVAILABLE_COMPONENTS } from '../constants';
import { Plus, Trash2, GripVertical, Activity, Settings, Map, ArrowDown, Layers } from 'lucide-react';

interface Props {
  config: SimulationConfig;
  setConfig: React.Dispatch<React.SetStateAction<SimulationConfig>>;
}

// Simple 2D Schematic for Well Architecture
const WellSchematic: React.FC<{ sections: WellSection[], maxDepth: number }> = ({ sections, maxDepth }) => {
  // Constants for drawing
  const width = 200;
  const height = 400;
  const scaleY = height / (maxDepth || 1000);
  const centerX = width / 2;
  const maxDiameter = Math.max(...sections.map(s => s.od), 24); // Normalize width
  const scaleX = (width * 0.8) / maxDiameter;

  return (
    <div className="h-full w-full flex flex-col items-center">
       <div className="bg-white border border-slate-200 rounded shadow-sm p-4 h-[450px] w-[240px] overflow-hidden relative">
         <div className="absolute top-2 right-2 text-[10px] text-slate-400 font-mono">MD: 0</div>
         <div className="absolute bottom-2 right-2 text-[10px] text-slate-400 font-mono">MD: {maxDepth.toFixed(0)}</div>
         
         <svg width="100%" height="100%" viewBox={`0 0 ${width} ${height}`} className="mx-auto">
            <defs>
              <pattern id="hatch" patternUnits="userSpaceOnUse" width="4" height="4" patternTransform="rotate(45)">
                <rect width="2" height="4" transform="translate(0,0)" fill="#cbd5e1" opacity="0.3" />
              </pattern>
            </defs>
            {/* Sort sections by diameter descending (outer first) so they stack correctly visually if overlapping */}
            {[...sections].sort((a,b) => b.od - a.od).map((section, idx) => {
               const top = section.md_top * scaleY;
               const h = (section.md_bottom - section.md_top) * scaleY;
               const w = section.od * scaleX;
               const x = centerX - (w / 2);
               
               return (
                 <g key={section.id}>
                   {/* Wall Left */}
                   <rect x={x} y={top} width={2} height={h} fill="#475569" />
                   {/* Wall Right */}
                   <rect x={x + w} y={top} width={2} height={h} fill="#475569" />
                   
                   {/* Fill between based on type */}
                   {section.type === SectionType.OPEN_HOLE ? (
                      <rect x={x} y={top} width={w} height={h} fill="url(#hatch)" />
                   ) : (
                      <rect x={x} y={top} width={w} height={h} fill="#94a3b8" fillOpacity={0.2} />
                   )}
                   
                   {/* Label if room */}
                   {h > 20 && (
                     <text x={centerX + w/2 + 5} y={top + h/2} fontSize="10" fill="#64748b" alignmentBaseline="middle">
                       {section.type === SectionType.OPEN_HOLE ? 'OH' : 'Csg'} {section.od}"
                     </text>
                   )}
                 </g>
               )
            })}
            {/* Center line */}
            <line x1={centerX} y1={0} x2={centerX} y2={height} stroke="#e2e8f0" strokeDasharray="4 4" />
         </svg>
       </div>
       <p className="text-xs text-slate-500 mt-2 font-medium">Wellbore Schematic (Telescoping View)</p>
    </div>
  );
};

export const ConfigurationPanel: React.FC<Props> = ({ config, setConfig }) => {
  const [activeTab, setActiveTab] = useState<'well' | 'string' | 'physics'>('well');

  // --- Well Path Handlers ---
  const updateWellPoint = (index: number, field: keyof WellPoint, value: number) => {
    const newPath = [...config.wellPath];
    newPath[index] = { ...newPath[index], [field]: value };
    setConfig({ ...config, wellPath: newPath });
  };

  const addWellPoint = () => {
    const last = config.wellPath[config.wellPath.length - 1];
    setConfig({
      ...config,
      wellPath: [...config.wellPath, { md: last.md + 100, inclination: last.inclination, azimuth: last.azimuth }]
    });
  };

  const removeWellPoint = (index: number) => {
    if (config.wellPath.length <= 2) return;
    const newPath = config.wellPath.filter((_, i) => i !== index);
    setConfig({ ...config, wellPath: newPath });
  };

  // --- Section Handlers ---
  const updateSection = (index: number, field: keyof WellSection, value: any) => {
    const newSections = [...config.sections];
    newSections[index] = { ...newSections[index], [field]: value };
    setConfig({ ...config, sections: newSections });
  };

  const addSection = () => {
    const last = config.sections[config.sections.length - 1];
    const newId = Math.random().toString(36).substr(2, 9);
    setConfig({
      ...config,
      sections: [...config.sections, { 
        id: newId, 
        name: 'New Section', 
        type: SectionType.CASING, 
        md_top: last ? last.md_bottom : 0, 
        md_bottom: last ? last.md_bottom + 500 : 500,
        od: last ? last.id_hole : 9.625,
        id_hole: last ? last.id_hole - 1 : 8.5
      }]
    });
  };

  const removeSection = (index: number) => {
    if (config.sections.length <= 1) return;
    const newSections = config.sections.filter((_, i) => i !== index);
    setConfig({ ...config, sections: newSections });
  };

  // --- String Handlers ---
  const addComponent = (comp: StringComponent) => {
    setConfig({ ...config, drillString: [...config.drillString, { ...comp, id: Math.random().toString() }] });
  };

  const removeComponent = (index: number) => {
    const newString = config.drillString.filter((_, i) => i !== index);
    setConfig({ ...config, drillString: newString });
  };

  // Max depth for schematic
  const maxDepth = useMemo(() => Math.max(
      ...config.wellPath.map(p => p.md),
      ...config.sections.map(s => s.md_bottom)
  ), [config]);

  return (
    <div className="flex flex-col h-full bg-slate-50 text-slate-900">
      {/* Tab Navigation */}
      <div className="flex border-b border-slate-200 bg-white">
        <button
          onClick={() => setActiveTab('well')}
          className={`flex items-center gap-2 px-6 py-4 text-sm font-medium transition-colors ${activeTab === 'well' ? 'bg-slate-50 text-cyan-600 border-b-2 border-cyan-600' : 'text-slate-500 hover:text-slate-800'}`}
        >
          <Map size={18} /> Well Architecture
        </button>
        <button
          onClick={() => setActiveTab('string')}
          className={`flex items-center gap-2 px-6 py-4 text-sm font-medium transition-colors ${activeTab === 'string' ? 'bg-slate-50 text-cyan-600 border-b-2 border-cyan-600' : 'text-slate-500 hover:text-slate-800'}`}
        >
          <ArrowDown size={18} /> Drill String
        </button>
        <button
          onClick={() => setActiveTab('physics')}
          className={`flex items-center gap-2 px-6 py-4 text-sm font-medium transition-colors ${activeTab === 'physics' ? 'bg-slate-50 text-cyan-600 border-b-2 border-cyan-600' : 'text-slate-500 hover:text-slate-800'}`}
        >
          <Activity size={18} /> Physics & Control
        </button>
      </div>

      <div className="flex-1 overflow-y-auto p-6">
        {/* WELL EDITOR */}
        {activeTab === 'well' && (
          <div className="grid grid-cols-1 xl:grid-cols-3 gap-6 max-w-7xl mx-auto">
            
            {/* Left Col: Trajectory */}
            <div className="xl:col-span-1 space-y-6">
               <div className="bg-white rounded-lg p-6 shadow-sm border border-slate-200">
                <div className="flex justify-between items-center mb-4">
                  <h3 className="text-lg font-semibold text-slate-800">Trajectory Points</h3>
                  <button onClick={addWellPoint} className="flex items-center gap-2 px-3 py-1.5 bg-cyan-600 hover:bg-cyan-500 text-white text-xs font-bold rounded transition-colors">
                    <Plus size={14} /> ADD
                  </button>
                </div>
                <div className="overflow-hidden rounded-md border border-slate-200">
                  <table className="w-full text-sm text-left text-slate-600">
                    <thead className="bg-slate-50 text-xs uppercase text-slate-500 border-b border-slate-200">
                      <tr>
                        <th className="px-3 py-2">MD</th>
                        <th className="px-3 py-2">Inc</th>
                        <th className="px-3 py-2">Azi</th>
                        <th className="px-3 py-2 text-right"></th>
                      </tr>
                    </thead>
                    <tbody className="divide-y divide-slate-100">
                      {config.wellPath.map((pt, idx) => (
                        <tr key={idx} className="hover:bg-slate-50">
                          <td className="px-3 py-2">
                            <input
                              type="number"
                              value={pt.md}
                              onChange={(e) => updateWellPoint(idx, 'md', parseFloat(e.target.value))}
                              className="bg-white border border-slate-200 rounded px-2 py-1 w-20 focus:border-cyan-500 focus:outline-none text-right"
                            />
                          </td>
                          <td className="px-3 py-2">
                            <input
                              type="number"
                              value={pt.inclination}
                              onChange={(e) => updateWellPoint(idx, 'inclination', parseFloat(e.target.value))}
                              className="bg-white border border-slate-200 rounded px-2 py-1 w-16 focus:border-cyan-500 focus:outline-none text-right"
                            />
                          </td>
                          <td className="px-3 py-2">
                            <input
                              type="number"
                              value={pt.azimuth}
                              onChange={(e) => updateWellPoint(idx, 'azimuth', parseFloat(e.target.value))}
                              className="bg-white border border-slate-200 rounded px-2 py-1 w-16 focus:border-cyan-500 focus:outline-none text-right"
                            />
                          </td>
                          <td className="px-3 py-2 text-right">
                            <button onClick={() => removeWellPoint(idx)} className="text-slate-400 hover:text-red-500 p-1">
                              <Trash2 size={14} />
                            </button>
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </div>
            </div>

            {/* Middle Col: Sections Table */}
            <div className="xl:col-span-1 space-y-6">
               <div className="bg-white rounded-lg p-6 shadow-sm border border-slate-200">
                <div className="flex justify-between items-center mb-4">
                  <h3 className="text-lg font-semibold text-slate-800">Well Architecture</h3>
                  <button onClick={addSection} className="flex items-center gap-2 px-3 py-1.5 bg-cyan-600 hover:bg-cyan-500 text-white text-xs font-bold rounded transition-colors">
                    <Plus size={14} /> ADD SECTION
                  </button>
                </div>
                <div className="space-y-3">
                  {config.sections.map((sec, idx) => (
                    <div key={sec.id} className="bg-slate-50 border border-slate-200 rounded p-3 space-y-2 text-sm">
                       <div className="flex justify-between items-center">
                          <input 
                             value={sec.name} 
                             onChange={e => updateSection(idx, 'name', e.target.value)}
                             className="font-semibold bg-transparent border-b border-transparent hover:border-slate-300 focus:border-cyan-500 focus:outline-none text-slate-800 w-full"
                          />
                          <button onClick={() => removeSection(idx)} className="text-slate-400 hover:text-red-500 ml-2"><Trash2 size={14} /></button>
                       </div>
                       <div className="grid grid-cols-2 gap-2">
                          <div>
                             <label className="text-[10px] uppercase text-slate-500">Type</label>
                             <select 
                               value={sec.type}
                               onChange={e => updateSection(idx, 'type', e.target.value)}
                               className="w-full bg-white border border-slate-200 rounded px-2 py-1 text-xs"
                             >
                               <option value={SectionType.CASING}>Casing</option>
                               <option value={SectionType.LINER}>Liner</option>
                               <option value={SectionType.OPEN_HOLE}>Open Hole</option>
                             </select>
                          </div>
                          <div>
                             <label className="text-[10px] uppercase text-slate-500">OD (inch)</label>
                             <input type="number" value={sec.od} onChange={e => updateSection(idx, 'od', parseFloat(e.target.value))} className="w-full bg-white border border-slate-200 rounded px-2 py-1" />
                          </div>
                          <div>
                             <label className="text-[10px] uppercase text-slate-500">Top MD</label>
                             <input type="number" value={sec.md_top} onChange={e => updateSection(idx, 'md_top', parseFloat(e.target.value))} className="w-full bg-white border border-slate-200 rounded px-2 py-1" />
                          </div>
                          <div>
                             <label className="text-[10px] uppercase text-slate-500">Bottom MD</label>
                             <input type="number" value={sec.md_bottom} onChange={e => updateSection(idx, 'md_bottom', parseFloat(e.target.value))} className="w-full bg-white border border-slate-200 rounded px-2 py-1" />
                          </div>
                       </div>
                    </div>
                  ))}
                </div>
               </div>
            </div>

            {/* Right Col: Schematic */}
            <div className="xl:col-span-1">
               <WellSchematic sections={config.sections} maxDepth={maxDepth} />
            </div>

          </div>
        )}

        {/* STRING EDITOR */}
        {activeTab === 'string' && (
          <div className="grid grid-cols-12 gap-6 h-full">
            {/* Library */}
            <div className="col-span-4 bg-white rounded-lg border border-slate-200 flex flex-col shadow-sm">
              <div className="p-4 border-b border-slate-100">
                <h3 className="font-semibold text-slate-800">Component Library</h3>
              </div>
              <div className="flex-1 overflow-y-auto p-4 space-y-3 bg-slate-50">
                {AVAILABLE_COMPONENTS.map((comp) => (
                  <div key={comp.id} className="bg-white p-3 rounded border border-slate-200 hover:border-cyan-500 hover:shadow-md transition-all cursor-pointer group" onClick={() => addComponent(comp)}>
                    <div className="flex justify-between items-center">
                      <span className="font-medium text-slate-700">{comp.name}</span>
                      <Plus size={16} className="text-cyan-600 opacity-0 group-hover:opacity-100" />
                    </div>
                    <div className="text-xs text-slate-500 mt-1 grid grid-cols-2 gap-2">
                      <span>OD: {comp.od}"</span>
                      <span>Len: {comp.length}m</span>
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Assembly */}
            <div className="col-span-8 bg-white rounded-lg border border-slate-200 flex flex-col shadow-sm">
              <div className="p-4 border-b border-slate-100 flex justify-between items-center">
                <h3 className="font-semibold text-slate-800">Current Assembly (Top to Bottom)</h3>
                <span className="text-xs text-slate-500 bg-slate-100 px-2 py-1 rounded">Total Length: {config.drillString.reduce((acc, c) => acc + c.length, 0).toFixed(2)} m</span>
              </div>
              <div className="flex-1 overflow-y-auto p-4 space-y-2 bg-slate-50/50">
                {config.drillString.length === 0 && (
                  <div className="h-full flex items-center justify-center text-slate-400 italic">
                    String is empty. Add components from the library.
                  </div>
                )}
                {config.drillString.map((comp, idx) => (
                  <div key={idx} className="flex items-center gap-4 bg-white p-3 rounded border border-slate-200 shadow-sm">
                    <div className="text-slate-400 text-xs font-mono w-6">#{idx + 1}</div>
                    <div className="flex-1">
                      <div className="font-medium text-slate-800">{comp.name}</div>
                      <div className="text-xs text-slate-500">
                         Type: <span className="capitalize">{comp.type.replace('_', ' ')}</span> | OD: {comp.od}" | ID: {comp.id_pipe}" | {comp.length}m
                      </div>
                    </div>
                    <button onClick={() => removeComponent(idx)} className="text-slate-400 hover:text-red-500 transition-colors">
                      <Trash2 size={16} />
                    </button>
                  </div>
                ))}
              </div>
            </div>
          </div>
        )}

        {/* PHYSICS EDITOR */}
        {activeTab === 'physics' && (
          <div className="grid grid-cols-2 gap-6 max-w-5xl mx-auto">
            <div className="bg-white p-6 rounded-lg border border-slate-200 shadow-sm">
              <div className="flex items-center gap-2 mb-6 text-cyan-600">
                <Settings size={20} />
                <h3 className="font-semibold text-slate-900">Operational Setpoints</h3>
              </div>
              <div className="space-y-4">
                <div className="grid grid-cols-2 items-center gap-4">
                  <label className="text-sm text-slate-600">Target RPM (Surface)</label>
                  <input
                    type="number"
                    value={config.operations.rpm_target}
                    onChange={e => setConfig({...config, operations: {...config.operations, rpm_target: parseFloat(e.target.value)}})}
                    className="bg-slate-50 border border-slate-200 rounded px-3 py-2 text-right focus:border-cyan-500 focus:outline-none"
                  />
                </div>
                <div className="grid grid-cols-2 items-center gap-4">
                  <label className="text-sm text-slate-600">Target WOB (kN)</label>
                  <input
                    type="number"
                    value={config.operations.wob_target}
                    onChange={e => setConfig({...config, operations: {...config.operations, wob_target: parseFloat(e.target.value)}})}
                    className="bg-slate-50 border border-slate-200 rounded px-3 py-2 text-right focus:border-cyan-500 focus:outline-none"
                  />
                </div>
                <div className="grid grid-cols-2 items-center gap-4">
                  <label className="text-sm text-slate-600">Target ROP (m/hr)</label>
                  <input
                    type="number"
                    value={config.operations.rop_target}
                    onChange={e => setConfig({...config, operations: {...config.operations, rop_target: parseFloat(e.target.value)}})}
                    className="bg-slate-50 border border-slate-200 rounded px-3 py-2 text-right focus:border-cyan-500 focus:outline-none"
                  />
                </div>
              </div>
            </div>

            <div className="space-y-6">
              <div className="bg-white p-6 rounded-lg border border-slate-200 shadow-sm">
                <h3 className="font-semibold text-slate-900 mb-4">Bit-Rock Interaction</h3>
                <div className="space-y-3">
                   <div className="flex justify-between items-center text-sm">
                     <span className="text-slate-500">Rock Strength (UCS)</span>
                     <span className="text-slate-900">{(config.bitRock.sigma / 1e6).toFixed(0)} MPa</span>
                   </div>
                   <input 
                      type="range" min="10" max="200" 
                      value={config.bitRock.sigma / 1e6} 
                      onChange={e => setConfig({...config, bitRock: {...config.bitRock, sigma: parseFloat(e.target.value) * 1e6}})}
                      className="w-full h-2 bg-slate-200 rounded-lg appearance-none cursor-pointer accent-cyan-600"
                   />
                   
                   <div className="flex justify-between items-center text-sm mt-4">
                     <span className="text-slate-500">Intrinsic Specific Energy</span>
                     <span className="text-slate-900">{(config.bitRock.epsilon / 1e6).toFixed(0)} MPa</span>
                   </div>
                   <input 
                      type="range" min="10" max="200" 
                      value={config.bitRock.epsilon / 1e6} 
                      onChange={e => setConfig({...config, bitRock: {...config.bitRock, epsilon: parseFloat(e.target.value) * 1e6}})}
                      className="w-full h-2 bg-slate-200 rounded-lg appearance-none cursor-pointer accent-cyan-600"
                   />
                </div>
              </div>

              <div className="bg-white p-6 rounded-lg border border-slate-200 shadow-sm">
                <h3 className="font-semibold text-slate-900 mb-4">Borehole Friction</h3>
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <label className="block text-xs text-slate-500 mb-1">Static Coeff</label>
                    <input
                      type="number" step="0.01"
                      value={config.contact.mu_static}
                      onChange={e => setConfig({...config, contact: {...config.contact, mu_static: parseFloat(e.target.value)}})}
                      className="w-full bg-slate-50 border border-slate-200 rounded px-2 py-1 text-sm focus:border-cyan-500 focus:outline-none"
                    />
                  </div>
                  <div>
                    <label className="block text-xs text-slate-500 mb-1">Kinetic Coeff</label>
                    <input
                      type="number" step="0.01"
                      value={config.contact.mu_kinetic}
                      onChange={e => setConfig({...config, contact: {...config.contact, mu_kinetic: parseFloat(e.target.value)}})}
                      className="w-full bg-slate-50 border border-slate-200 rounded px-2 py-1 text-sm focus:border-cyan-500 focus:outline-none"
                    />
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};