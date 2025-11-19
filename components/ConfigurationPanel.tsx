
import React, { useState, useMemo } from 'react';
import { SimulationConfig, WellPoint, StringComponent, ComponentType, SectionType, WellSection } from '../types';
import { AVAILABLE_COMPONENTS } from '../constants';
import { Plus, Trash2, GripVertical, Activity, Map, ArrowDown, X, Info, AlertTriangle, Edit2, Hammer, Sliders, Zap } from 'lucide-react';

interface Props {
  config: SimulationConfig;
  setConfig: React.Dispatch<React.SetStateAction<SimulationConfig>>;
}

// Validation Function
const validateWellArchitecture = (sections: WellSection[]): string[] => {
  const errors: string[] = [];
  const sorted = [...sections].sort((a, b) => a.md_bottom - b.md_bottom);
  
  for (let i = 0; i < sorted.length; i++) {
    const current = sorted[i];
    if (current.type !== SectionType.OPEN_HOLE && current.id_hole >= current.od) {
      errors.push(`Section "${current.name}": Inner ID (${current.id_hole}") cannot be larger than or equal to OD (${current.od}").`);
    }
    if (i > 0) {
      const parentSection = sorted[i-1];
      if (parentSection) {
        // Allow equal diameters (e.g. open hole drilled same size as previous ID)
        if (current.od > parentSection.id_hole) {
           errors.push(`Clearance Issue: "${current.name}" (OD ${current.od}") will not fit inside "${parentSection.name}" (ID ${parentSection.id_hole}").`);
        }
      }
    }
  }
  return errors;
};

const safeParseFloat = (val: string, fallback = 0): number => {
  const num = parseFloat(val);
  return isNaN(num) ? fallback : num;
};

export const ConfigurationPanel: React.FC<Props> = ({ config, setConfig }) => {
  const [activeTab, setActiveTab] = useState<'well' | 'string' | 'physics'>('well');
  const [draggedItemIndex, setDraggedItemIndex] = useState<number | null>(null);
  const [selectedComponentId, setSelectedComponentId] = useState<string | null>(null);

  const validationErrors = useMemo(() => validateWellArchitecture(config.sections), [config.sections]);

  // --- Well & Section Handlers ---
  const updateWellPoint = (index: number, field: keyof WellPoint, value: number) => {
    if (isNaN(value)) return; 
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

  const updateSection = (index: number, field: keyof WellSection, value: any) => {
    const newSections = [...config.sections];
    const prevSection = index > 0 ? newSections[index - 1] : null;
    
    let updatedSection = { ...newSections[index], [field]: value };
    const val = typeof value === 'number' ? value : parseFloat(value);

    if (updatedSection.type === SectionType.OPEN_HOLE) {
        // For Open Hole: OD and ID are the same (Hole Size)
        if (field === 'od' || field === 'id_hole') {
            updatedSection.od = val;
            updatedSection.id_hole = val;
            
            // Auto-clamp to previous ID to ensure it fits
            if (prevSection && val > prevSection.id_hole) {
                updatedSection.od = prevSection.id_hole;
                updatedSection.id_hole = prevSection.id_hole;
            }
        }
    } else {
        // Casing / Liner Logic
        if (field === 'od') {
            // Constraint: OD must be > ID
            // If new OD is <= ID, automatically push ID down
            if (val <= updatedSection.id_hole) {
                updatedSection.id_hole = Number((val * 0.88).toFixed(3)); // Default 88% drift
            }
        }
        if (field === 'id_hole') {
            // Constraint: ID must be < OD
            // If new ID is >= OD, automatically push OD up
            if (val >= updatedSection.od) {
                updatedSection.od = Number((val * 1.12).toFixed(3));
            }
        }
    }

    newSections[index] = updatedSection;
    setConfig({ ...config, sections: newSections });
  };

  const addSection = () => {
    const last = config.sections[config.sections.length - 1];
    const newId = Math.random().toString(36).substr(2, 9);
    const defaultOD = last ? last.id_hole * 0.9 : 9.625; 
    const defaultID = defaultOD * 0.85; 

    setConfig({
      ...config,
      sections: [...config.sections, { 
        id: newId, 
        name: 'New Section', 
        type: SectionType.CASING, 
        md_top: last ? last.md_bottom : 0, 
        md_bottom: last ? last.md_bottom + 500 : 500,
        od: parseFloat(defaultOD.toFixed(3)),
        id_hole: parseFloat(defaultID.toFixed(3))
      }]
    });
  };

  const removeSection = (index: number) => {
    if (config.sections.length <= 1) return;
    const newSections = config.sections.filter((_, i) => i !== index);
    setConfig({ ...config, sections: newSections });
  };

  // --- Drill String Handlers ---
  const addComponent = (comp: StringComponent) => {
    if (comp.type === ComponentType.BIT) {
      if (config.drillString.some(c => c.type === ComponentType.BIT)) {
         alert("The assembly already has a bit.");
         return;
      }
      const newBit = { ...comp, id: Math.random().toString(), count: 1 };
      setConfig({ ...config, drillString: [...config.drillString, newBit] });
      setSelectedComponentId(newBit.id);
      return;
    }
    const newComp = { ...comp, id: Math.random().toString(), count: 1 };
    const hasBit = config.drillString.length > 0 && config.drillString[config.drillString.length - 1].type === ComponentType.BIT;
    
    let newString;
    if (hasBit) {
       const bit = config.drillString[config.drillString.length - 1];
       const others = config.drillString.slice(0, -1);
       newString = [...others, newComp, bit];
    } else {
       newString = [...config.drillString, newComp];
    }
    setConfig({ ...config, drillString: newString });
    setSelectedComponentId(newComp.id);
  };

  const removeComponent = (e: React.MouseEvent, index: number) => {
    e.stopPropagation();
    const targetId = config.drillString[index].id;
    const newString = config.drillString.filter((_, i) => i !== index);
    setConfig({ ...config, drillString: newString });
    if (selectedComponentId === targetId) setSelectedComponentId(null);
  };

  const updateComponent = (id: string, field: keyof StringComponent, value: any) => {
    const index = config.drillString.findIndex(c => c.id === id);
    if (index === -1) return;
    const newString = [...config.drillString];
    newString[index] = { ...newString[index], [field]: value };
    setConfig({ ...config, drillString: newString });
  };

  const updateStabilizerParams = (id: string, field: string, value: number) => {
    const index = config.drillString.findIndex(c => c.id === id);
    if (index === -1) return;
    const comp = config.drillString[index];
    const newStab = { ...comp.stabilizer!, [field]: value };
    const newString = [...config.drillString];
    newString[index] = { ...comp, stabilizer: newStab };
    setConfig({ ...config, drillString: newString });
  };

  // --- Physics Handlers ---
  const updateOperations = (field: string, value: number) => {
    setConfig({ ...config, operations: { ...config.operations, [field]: value }});
  };

  const updateContact = (field: string, value: number) => {
    setConfig({ ...config, contact: { ...config.contact, [field]: value }});
  };

  const updateBitRock = (field: string, value: number) => {
    setConfig({ ...config, bitRock: { ...config.bitRock, [field]: value }});
  };

  // Drag and Drop
  const handleDragStart = (e: React.DragEvent, index: number) => {
    if (config.drillString[index].type === ComponentType.BIT) {
        e.preventDefault();
        return;
    }
    setDraggedItemIndex(index);
    e.dataTransfer.effectAllowed = "move";
  };

  const handleDragOver = (e: React.DragEvent, index: number) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = "move";
  };

  const handleDrop = (e: React.DragEvent, index: number) => {
    e.preventDefault();
    if (draggedItemIndex === null || draggedItemIndex === index) return;
    
    const newString = [...config.drillString];
    const [draggedItem] = newString.splice(draggedItemIndex, 1);
    newString.splice(index, 0, draggedItem);
    
    const bitIndex = newString.findIndex(c => c.type === ComponentType.BIT);
    if (bitIndex !== -1 && bitIndex !== newString.length - 1) {
        const [bit] = newString.splice(bitIndex, 1);
        newString.push(bit);
    }

    setConfig({ ...config, drillString: newString });
    setDraggedItemIndex(null);
  };

  const selectedComponent = config.drillString.find(c => c.id === selectedComponentId);

  return (
    <div className="flex flex-col h-full bg-slate-50 text-slate-900">
      <div className="flex border-b border-slate-200 bg-white overflow-x-auto scrollbar-hide shrink-0">
        <button onClick={() => setActiveTab('well')} className={`flex items-center gap-2 px-6 py-4 text-sm font-medium transition-colors whitespace-nowrap ${activeTab === 'well' ? 'bg-slate-50 text-cyan-600 border-b-2 border-cyan-600' : 'text-slate-500 hover:text-slate-800'}`}>
          <Map size={18} /> Well Arch
        </button>
        <button onClick={() => setActiveTab('string')} className={`flex items-center gap-2 px-6 py-4 text-sm font-medium transition-colors whitespace-nowrap ${activeTab === 'string' ? 'bg-slate-50 text-cyan-600 border-b-2 border-cyan-600' : 'text-slate-500 hover:text-slate-800'}`}>
          <ArrowDown size={18} /> Drill String
        </button>
        <button onClick={() => setActiveTab('physics')} className={`flex items-center gap-2 px-6 py-4 text-sm font-medium transition-colors whitespace-nowrap ${activeTab === 'physics' ? 'bg-slate-50 text-cyan-600 border-b-2 border-cyan-600' : 'text-slate-500 hover:text-slate-800'}`}>
          <Activity size={18} /> Physics
        </button>
      </div>

      <div className="flex-1 overflow-hidden p-4 flex flex-col">
        {/* WELL EDITOR */}
        {activeTab === 'well' && (
          <div className="overflow-y-auto h-full pr-2 space-y-6">
             {validationErrors.length > 0 && (
              <div className="bg-red-50 border border-red-200 rounded-md p-4">
                <div className="flex items-center gap-2 text-red-700 font-semibold mb-2"><AlertTriangle size={18} /><span>Validation Issues</span></div>
                <ul className="list-disc list-inside text-xs text-red-600">{validationErrors.map((err, i) => <li key={i}>{err}</li>)}</ul>
              </div>
            )}
            
            {/* Well Sections */}
            <div className="bg-white rounded-lg p-4 shadow-sm border border-slate-200">
                <div className="flex justify-between items-center mb-4">
                  <h3 className="font-semibold text-slate-800">Well Sections</h3>
                  <button onClick={addSection} className="px-2 py-1 bg-cyan-600 text-white text-xs rounded hover:bg-cyan-700 transition-colors"><Plus size={14} /></button>
                </div>
                <div className="space-y-3">
                  {config.sections.map((sec, idx) => (
                    <div key={sec.id} className="bg-slate-50 border border-slate-200 rounded-lg p-4">
                       {/* Section Header */}
                       <div className="flex justify-between items-start mb-4">
                          <div className="flex-1 mr-4">
                              <label className="block text-[10px] font-bold text-slate-500 uppercase mb-1">Section Name</label>
                              <input 
                                value={sec.name} 
                                onChange={e => updateSection(idx, 'name', e.target.value)} 
                                className="w-full p-2 bg-white border border-slate-300 rounded text-sm font-semibold text-slate-700 focus:border-cyan-500 focus:ring-1 focus:ring-cyan-500 outline-none" 
                              />
                          </div>
                          {config.sections.length > 1 && (
                             <button onClick={() => removeSection(idx)} className="mt-6 text-slate-400 hover:text-red-500 transition-colors p-1"><Trash2 size={16} /></button>
                          )}
                       </div>

                       {/* Properties Grid */}
                       <div className="grid grid-cols-12 gap-3">
                           <div className="col-span-6 sm:col-span-3">
                               <label className="block text-[10px] font-bold text-slate-500 uppercase mb-1">Top (m)</label>
                               <input type="number" value={sec.md_top} onChange={e => updateSection(idx, 'md_top', safeParseFloat(e.target.value))} className="w-full p-2 bg-white border border-slate-300 rounded text-sm text-slate-700" />
                           </div>
                           <div className="col-span-6 sm:col-span-3">
                               <label className="block text-[10px] font-bold text-slate-500 uppercase mb-1">Bottom (m)</label>
                               <input type="number" value={sec.md_bottom} onChange={e => updateSection(idx, 'md_bottom', safeParseFloat(e.target.value))} className="w-full p-2 bg-white border border-slate-300 rounded text-sm text-slate-700" />
                           </div>

                           {sec.type === SectionType.OPEN_HOLE ? (
                             <div className="col-span-12 sm:col-span-6">
                                <label className="block text-[10px] font-bold text-slate-500 uppercase mb-1">Hole Diameter (in)</label>
                                <div className="relative">
                                  <input 
                                    type="number" 
                                    value={sec.od} 
                                    onChange={e => updateSection(idx, 'od', safeParseFloat(e.target.value))} 
                                    className="w-full p-2 bg-white border border-slate-300 rounded text-sm font-mono text-slate-800 focus:border-cyan-500 outline-none" 
                                  />
                                  <span className="absolute right-3 top-2.5 text-xs text-slate-400 pointer-events-none">in</span>
                                </div>
                                <p className="text-[10px] text-slate-400 mt-1">Diameter automatically constrained to previous section ID.</p>
                             </div>
                           ) : (
                             <>
                                <div className="col-span-6 sm:col-span-3">
                                    <label className="block text-[10px] font-bold text-slate-500 uppercase mb-1">OD (in)</label>
                                    <input type="number" value={sec.od} onChange={e => updateSection(idx, 'od', safeParseFloat(e.target.value))} className="w-full p-2 bg-white border border-slate-300 rounded text-sm font-mono text-cyan-700 font-medium" />
                                </div>
                                <div className="col-span-6 sm:col-span-3">
                                    <label className="block text-[10px] font-bold text-slate-500 uppercase mb-1">ID (in)</label>
                                    <input type="number" value={sec.id_hole} onChange={e => updateSection(idx, 'id_hole', safeParseFloat(e.target.value))} className="w-full p-2 bg-white border border-slate-300 rounded text-sm font-mono text-emerald-700 font-medium" />
                                </div>
                             </>
                           )}
                       </div>
                    </div>
                  ))}
                </div>
            </div>

            {/* Trajectory */}
            <div className="bg-white rounded-lg p-4 shadow-sm border border-slate-200">
                 <div className="flex justify-between items-center mb-4">
                  <h3 className="font-semibold text-slate-800">Trajectory</h3>
                  <button onClick={addWellPoint} className="px-2 py-1 bg-cyan-600 text-white text-xs rounded hover:bg-cyan-700"><Plus size={14} /></button>
                </div>
                <div className="bg-slate-50 rounded border border-slate-200 overflow-hidden">
                   <table className="w-full text-xs">
                      <thead className="bg-slate-100 text-slate-500 border-b border-slate-200">
                         <tr>
                            <th className="px-2 py-2 text-left font-medium">MD (m)</th>
                            <th className="px-2 py-2 text-left font-medium">Inc (°)</th>
                            <th className="px-2 py-2 text-left font-medium">Azi (°)</th>
                            <th className="px-2 py-2 w-8"></th>
                         </tr>
                      </thead>
                      <tbody className="divide-y divide-slate-200">
                         {config.wellPath.map((pt, idx) => (
                            <tr key={idx} className="group hover:bg-white transition-colors">
                               <td className="p-1">
                                  <input 
                                     type="number" 
                                     value={pt.md} 
                                     onChange={e => updateWellPoint(idx, 'md', safeParseFloat(e.target.value))} 
                                     className="w-full bg-transparent border border-transparent hover:border-slate-300 rounded px-1 focus:border-cyan-500 focus:ring-0"
                                  />
                               </td>
                               <td className="p-1">
                                  <input 
                                     type="number" 
                                     value={pt.inclination} 
                                     onChange={e => updateWellPoint(idx, 'inclination', safeParseFloat(e.target.value))} 
                                     className="w-full bg-transparent border border-transparent hover:border-slate-300 rounded px-1 focus:border-cyan-500 focus:ring-0"
                                  />
                               </td>
                               <td className="p-1">
                                  <input 
                                     type="number" 
                                     value={pt.azimuth} 
                                     onChange={e => updateWellPoint(idx, 'azimuth', safeParseFloat(e.target.value))} 
                                     className="w-full bg-transparent border border-transparent hover:border-slate-300 rounded px-1 focus:border-cyan-500 focus:ring-0"
                                  />
                               </td>
                               <td className="p-1 text-center">
                                  <button onClick={() => removeWellPoint(idx)} className="text-slate-300 hover:text-red-500 transition-colors"><Trash2 size={12} /></button>
                               </td>
                            </tr>
                         ))}
                      </tbody>
                   </table>
                </div>
            </div>
          </div>
        )}

        {/* STRING EDITOR */}
        {activeTab === 'string' && (
          <div className="flex flex-col h-full gap-4">
            {/* Top Area: Component Selection & Assembly List */}
            <div className="flex-1 flex gap-4 min-h-0">
               {/* Library */}
               <div className="w-1/3 flex flex-col bg-white rounded-lg border border-slate-200 shadow-sm">
                 <div className="p-2 border-b border-slate-100 font-semibold text-xs text-slate-600 uppercase bg-slate-50 rounded-t-lg">Library</div>
                 <div className="flex-1 overflow-y-auto p-2 space-y-2">
                    {AVAILABLE_COMPONENTS.map((comp) => (
                      <div key={comp.id} onClick={() => addComponent(comp)} className="bg-white p-2 rounded border border-slate-200 hover:border-cyan-500 cursor-pointer text-xs shadow-sm group">
                         <div className="font-medium text-slate-700 group-hover:text-cyan-700">{comp.name}</div>
                         <div className="text-[10px] text-slate-400">OD: {comp.od}"</div>
                      </div>
                    ))}
                 </div>
               </div>

               {/* Current Assembly List */}
               <div className="flex-1 flex flex-col bg-white rounded-lg border border-slate-200 shadow-sm">
                  <div className="p-2 border-b border-slate-100 flex justify-between items-center bg-slate-50 rounded-t-lg">
                    <span className="font-semibold text-xs text-slate-600 uppercase">Drill String Assembly</span>
                    <span className="text-[10px] bg-slate-200 px-2 py-0.5 rounded text-slate-600">
                       {config.drillString.length} items
                    </span>
                  </div>
                  <div className="flex-1 overflow-y-auto p-2 space-y-1 bg-slate-100/50">
                    {config.drillString.map((comp, idx) => {
                      const isSelected = selectedComponentId === comp.id;
                      return (
                        <div 
                          key={comp.id}
                          onClick={() => setSelectedComponentId(comp.id)}
                          draggable={comp.type !== ComponentType.BIT}
                          onDragStart={(e) => handleDragStart(e, idx)}
                          onDragOver={(e) => handleDragOver(e, idx)}
                          onDrop={(e) => handleDrop(e, idx)}
                          className={`flex items-center gap-2 p-2 rounded border text-xs cursor-pointer transition-all ${
                            isSelected 
                              ? 'bg-cyan-50 border-cyan-500 ring-1 ring-cyan-500 z-10' 
                              : 'bg-white border-slate-200 hover:border-cyan-300'
                          } ${draggedItemIndex === idx ? 'opacity-50' : ''}`}
                        >
                          <div className="text-slate-400 cursor-grab"><GripVertical size={12} /></div>
                          <div className="font-mono text-slate-400 w-4 text-center">{idx + 1}</div>
                          <div className="flex-1 font-medium text-slate-700">{comp.name}</div>
                          <div className="text-slate-500">{comp.od}" OD</div>
                          <div className="bg-slate-100 px-1.5 py-0.5 rounded text-[10px] font-mono font-bold text-slate-600">x{comp.count}</div>
                          <button onClick={(e) => removeComponent(e, idx)} className="text-slate-300 hover:text-red-500"><X size={14} /></button>
                        </div>
                      );
                    })}
                  </div>
               </div>
            </div>

            {/* Bottom Area: Property Inspector */}
            <div className="h-[200px] bg-white rounded-lg border border-slate-200 shadow-sm flex flex-col shrink-0">
              <div className="p-2 border-b border-slate-100 bg-slate-50 rounded-t-lg flex items-center gap-2">
                 <Edit2 size={12} className="text-cyan-600" />
                 <span className="font-semibold text-xs text-slate-600 uppercase">Component Properties</span>
              </div>
              
              {selectedComponent ? (
                <div className="p-4 overflow-y-auto grid grid-cols-4 gap-4 content-start">
                   <div className="col-span-4 mb-2 pb-2 border-b border-slate-100 flex justify-between items-center">
                      <input 
                        type="text" 
                        value={selectedComponent.name} 
                        onChange={(e) => updateComponent(selectedComponent.id, 'name', e.target.value)}
                        className="font-bold text-lg text-slate-800 bg-transparent border-none focus:ring-0 p-0 w-full"
                      />
                      <span className="text-xs px-2 py-1 bg-slate-100 rounded text-slate-500 uppercase">{selectedComponent.type.replace('_', ' ')}</span>
                   </div>

                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Quantity</label>
                      <input 
                        type="number" 
                        min="1" 
                        value={selectedComponent.count} 
                        onChange={(e) => updateComponent(selectedComponent.id, 'count', safeParseFloat(e.target.value))}
                        className="w-full border border-slate-200 rounded p-1.5 text-sm bg-slate-50 focus:border-cyan-500 outline-none transition-colors"
                        disabled={[ComponentType.BIT, ComponentType.JAR, ComponentType.SUB].includes(selectedComponent.type)}
                      />
                   </div>

                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Length (m)</label>
                      <input 
                        type="number" 
                        value={selectedComponent.length} 
                        onChange={(e) => updateComponent(selectedComponent.id, 'length', safeParseFloat(e.target.value))}
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                   </div>

                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Body OD (in)</label>
                      <input 
                        type="number" 
                        value={selectedComponent.od} 
                        onChange={(e) => updateComponent(selectedComponent.id, 'od', safeParseFloat(e.target.value))}
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none text-cyan-700 font-semibold"
                      />
                   </div>

                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Pipe ID (in)</label>
                      <input 
                        type="number" 
                        value={selectedComponent.id_pipe} 
                        onChange={(e) => updateComponent(selectedComponent.id, 'id_pipe', safeParseFloat(e.target.value))}
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none text-emerald-700 font-semibold"
                      />
                   </div>

                   {selectedComponent.type === ComponentType.STABILIZER && selectedComponent.stabilizer && (
                      <div className="col-span-4 mt-2 bg-slate-50 p-3 rounded border border-slate-200 grid grid-cols-3 gap-4 relative">
                         <div className="absolute -top-2 left-2 bg-slate-50 px-1 text-[10px] text-slate-500 font-bold">STABILIZER BLADE CONFIG</div>
                         <div className="space-y-1">
                            <label className="text-[10px] font-bold text-slate-500">Blade OD (in)</label>
                            <input 
                              type="number" 
                              value={selectedComponent.stabilizer.bladeOd} 
                              onChange={(e) => updateStabilizerParams(selectedComponent.id, 'bladeOd', safeParseFloat(e.target.value))}
                              className="w-full bg-white border border-slate-200 rounded p-1 text-xs"
                            />
                         </div>
                         <div className="space-y-1">
                            <label className="text-[10px] font-bold text-slate-500">Blade Length (m)</label>
                            <input 
                              type="number" 
                              value={selectedComponent.stabilizer.bladeLength} 
                              onChange={(e) => updateStabilizerParams(selectedComponent.id, 'bladeLength', safeParseFloat(e.target.value))}
                              className="w-full bg-white border border-slate-200 rounded p-1 text-xs"
                            />
                         </div>
                         <div className="space-y-1">
                            <label className="text-[10px] font-bold text-slate-500">Dist from Bottom (m)</label>
                            <input 
                              type="number" 
                              value={selectedComponent.stabilizer.distFromBottom} 
                              onChange={(e) => updateStabilizerParams(selectedComponent.id, 'distFromBottom', safeParseFloat(e.target.value))}
                              className="w-full bg-white border border-slate-200 rounded p-1 text-xs"
                            />
                         </div>
                      </div>
                   )}
                </div>
              ) : (
                <div className="flex-1 flex flex-col items-center justify-center text-slate-400 p-4">
                   <Info size={24} className="mb-2 opacity-50" />
                   <p className="text-xs">Select a component from the list above to edit properties.</p>
                </div>
              )}
            </div>

          </div>
        )}

        {/* PHYSICS EDITOR */}
        {activeTab === 'physics' && (
           <div className="overflow-y-auto h-full space-y-4 p-1">
              
              {/* Operational Parameters */}
              <div className="bg-white p-4 rounded-lg border border-slate-200 shadow-sm">
                <div className="flex items-center gap-2 mb-4 border-b border-slate-100 pb-2">
                   <Sliders size={16} className="text-cyan-600"/>
                   <h3 className="font-bold text-slate-700 text-sm uppercase">Operational Parameters</h3>
                </div>
                <div className="grid grid-cols-2 gap-4">
                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Target RPM</label>
                      <input 
                        type="number" 
                        value={config.operations.rpm_target} 
                        onChange={e => updateOperations('rpm_target', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                   </div>
                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Target WOB (kN)</label>
                      <input 
                        type="number" 
                        value={config.operations.wob_target} 
                        onChange={e => updateOperations('wob_target', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                   </div>
                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Top Feed Rate / ROP (m/hr)</label>
                      <input 
                        type="number" 
                        value={config.operations.rop_target} 
                        onChange={e => updateOperations('rop_target', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                   </div>
                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Flow Rate (L/min)</label>
                      <input 
                        type="number" 
                        value={config.operations.flow_rate} 
                        onChange={e => updateOperations('flow_rate', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                   </div>
                </div>
              </div>

              {/* Contact Physics */}
              <div className="bg-white p-4 rounded-lg border border-slate-200 shadow-sm">
                <div className="flex items-center gap-2 mb-4 border-b border-slate-100 pb-2">
                   <Activity size={16} className="text-orange-500"/>
                   <h3 className="font-bold text-slate-700 text-sm uppercase">Wellbore Friction</h3>
                </div>
                <div className="grid grid-cols-2 gap-4">
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Static Friction (μ)</label>
                      <input 
                        type="number" 
                        step="0.01"
                        value={config.contact.mu_static} 
                        onChange={e => updateContact('mu_static', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                  </div>
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Kinetic Friction (μ)</label>
                      <input 
                        type="number" 
                        step="0.01"
                        value={config.contact.mu_kinetic} 
                        onChange={e => updateContact('mu_kinetic', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                  </div>
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Stribeck Velocity (m/s)</label>
                      <input 
                        type="number" 
                        step="0.01"
                        value={config.contact.stribeck_velocity} 
                        onChange={e => updateContact('stribeck_velocity', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                  </div>
                </div>
              </div>

              {/* Bit / Rock Mechanics */}
              <div className="bg-white p-4 rounded-lg border border-slate-200 shadow-sm">
                <div className="flex items-center gap-2 mb-4 border-b border-slate-100 pb-2">
                   <Hammer size={16} className="text-purple-600"/>
                   <h3 className="font-bold text-slate-700 text-sm uppercase">Bit / Rock Mechanics</h3>
                </div>
                <div className="grid grid-cols-2 gap-4">
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Rock Strength (Pa)</label>
                      <input 
                        type="number" 
                        value={config.bitRock.sigma} 
                        onChange={e => updateBitRock('sigma', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                      <div className="text-[10px] text-slate-400 text-right">{(config.bitRock.sigma / 1e6).toFixed(1)} MPa</div>
                  </div>
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Specific Energy (Pa)</label>
                      <input 
                        type="number" 
                        value={config.bitRock.epsilon} 
                        onChange={e => updateBitRock('epsilon', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                       <div className="text-[10px] text-slate-400 text-right">{(config.bitRock.epsilon / 1e6).toFixed(1)} MPa</div>
                  </div>
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Bit Friction (μ)</label>
                      <input 
                        type="number" 
                        step="0.01"
                        value={config.bitRock.mu} 
                        onChange={e => updateBitRock('mu', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                  </div>
                  <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Bit Gamma</label>
                      <input 
                        type="number" 
                        step="0.1"
                        value={config.bitRock.gamma} 
                        onChange={e => updateBitRock('gamma', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                  </div>
                   <div className="space-y-1">
                      <label className="text-[10px] font-bold text-slate-500 uppercase">Blade Count</label>
                      <input 
                        type="number" 
                        step="1"
                        value={config.bitRock.n_blades} 
                        onChange={e => updateBitRock('n_blades', safeParseFloat(e.target.value))} 
                        className="w-full border border-slate-200 rounded p-1.5 text-sm focus:border-cyan-500 outline-none"
                      />
                  </div>
                </div>
              </div>

           </div>
        )}
      </div>
    </div>
  );
};
