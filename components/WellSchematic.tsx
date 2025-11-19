
import React from 'react';
import { WellSection, SectionType, StringComponent, ComponentType } from '../types';

interface Props {
  sections: WellSection[];
  maxDepth: number;
  drillString?: StringComponent[];
  rpm?: number;
}

const WellSchematicComponent: React.FC<Props> = ({ sections, maxDepth, drillString = [], rpm = 0 }) => {
  // Dimensions
  const width = 600;
  const height = 800; 
  const paddingY = 60;
  const drawHeight = height - (paddingY * 2);
  
  // Scale Calculations
  const maxMD = Math.max(maxDepth, 100); 
  const scaleY = drawHeight / maxMD;
  
  // X Scale: Calculate based on largest OD in the entire system
  const maxHoleOD = Math.max(...sections.map((s: WellSection) => s.od), 30); 
  const maxStringOD = Math.max(...drillString.map((c: StringComponent) => c.stabilizer?.bladeOd || c.od), 10);
  const largestOD = Math.max(maxHoleOD, maxStringOD) * 1.2; 
  
  const scaleX = (width * 0.5) / largestOD; 
  const centerX = width / 2;

  // Sort sections largest OD to smallest (Painter's algo for bodies)
  const sortedSections = [...sections].sort((a, b) => b.od - a.od);

  const depthMarkers: number[] = Array.from<number>(new Set(
    sections.flatMap(s => [s.md_top, s.md_bottom])
  )).sort((a, b) => a - b);
  
  const uniqueDepths = depthMarkers.filter((d: number) => d <= maxMD);

  // Drill String Positioning
  const wellTotalDepth = Math.max(...sections.map(s => s.md_bottom), 100);
  const stringTotalLength = drillString.reduce((acc: number, c: StringComponent) => acc + (c.length * (c.count || 1)), 0);
  
  // If we are drilling (simulating), and the bit is deeper than hole, we clamp it. 
  // But usually in sim the hole gets deeper. For this view, we align bit to bottom or string length.
  const bitDepth = stringTotalLength > wellTotalDepth ? wellTotalDepth : stringTotalLength;
  
  const isRotating = rpm > 10;
  const animationDuration = isRotating ? Math.max(0.2, 60 / rpm) : 0; // seconds per cycle

  return (
    <div className="w-full h-full bg-white overflow-auto flex justify-center border-l border-slate-200 relative select-none cursor-crosshair">
      <svg width={width} height={height} className="my-4 shadow-sm bg-white">
        <defs>
          <linearGradient id="steelGradient" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="#cbd5e1" />
            <stop offset="20%" stopColor="#e2e8f0" />
            <stop offset="50%" stopColor="#f8fafc" />
            <stop offset="80%" stopColor="#e2e8f0" />
            <stop offset="100%" stopColor="#94a3b8" />
          </linearGradient>
          
          {/* Clean Engineering Grid Pattern */}
          <pattern id="grid" width="50" height="50" patternUnits="userSpaceOnUse">
            <path d="M 50 0 L 0 0 0 50" fill="none" stroke="#f1f5f9" strokeWidth="1"/>
          </pattern>

          {/* Stabilizer Pattern (Diagonal Stripes) - Animated */}
          <pattern id="stabPattern" width="10" height="10" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
            <rect width="10" height="10" fill="#475569" />
            <line x1="0" y1="0" x2="0" y2="10" stroke="#334155" strokeWidth="3" />
            {isRotating && (
               <animate attributeName="y" from="0" to="10" dur={`${animationDuration * 0.5}s`} repeatCount="indefinite" />
            )}
          </pattern>

          {/* Bit Face Pattern (Cutters) - Animated */}
          <pattern id="bitPattern" width="8" height="8" patternUnits="userSpaceOnUse">
             <rect width="8" height="8" fill="#1e293b" />
             <circle cx="4" cy="4" r="2" fill="#94a3b8" />
             {isRotating && (
                <animate attributeName="x" from="0" to="8" dur={`${animationDuration}s`} repeatCount="indefinite" />
             )}
          </pattern>
        </defs>

        {/* 1. Background */}
        <rect x="0" y={paddingY} width={width} height={drawHeight} fill="url(#grid)" />
        
        {/* Mudline */}
        <line x1={0} y1={paddingY} x2={width} y2={paddingY} stroke="#854d0e" strokeWidth="4" />
        <text x={10} y={paddingY - 10} fontSize="12" fontWeight="bold" fill="#854d0e">MUDLINE / SURFACE</text>

        {/* 2. Casing Bodies */}
        {sortedSections.map((section) => {
          const topY = paddingY + (section.md_top * scaleY);
          const bottomY = paddingY + (section.md_bottom * scaleY);
          const heightPx = bottomY - topY;
          const rOuter = (section.od / 2) * scaleX;
          const fill = section.type === SectionType.OPEN_HOLE ? '#fff' : '#d1d5db';
          const opacity = section.type === SectionType.OPEN_HOLE ? 0.5 : 1.0;

          return (
            <g key={`body-${section.id}`}>
              <rect x={centerX - rOuter} y={topY} width={rOuter * 2} height={heightPx} fill={fill} opacity={opacity} />
              {section.type !== SectionType.OPEN_HOLE && (
                <>
                  <line x1={centerX - rOuter} y1={topY} x2={centerX - rOuter} y2={bottomY} stroke="#000" strokeWidth="1.5" />
                  <line x1={centerX + rOuter} y1={topY} x2={centerX + rOuter} y2={bottomY} stroke="#000" strokeWidth="1.5" />
                </>
              )}
              {section.type === SectionType.OPEN_HOLE && (
                <>
                   <line x1={centerX - rOuter} y1={topY} x2={centerX - rOuter} y2={bottomY} stroke="#64748b" strokeWidth="1" strokeDasharray="6 4" />
                   <line x1={centerX + rOuter} y1={topY} x2={centerX + rOuter} y2={bottomY} stroke="#64748b" strokeWidth="1" strokeDasharray="6 4" />
                </>
              )}
            </g>
          );
        })}

        {/* 3. Inner Void */}
        {uniqueDepths.map((depth, i) => {
           if (i === uniqueDepths.length - 1) return null;
           const startMd = depth;
           const endMd = uniqueDepths[i+1];
           
           if (typeof endMd !== 'number') return null;

           const midMd = (startMd + endMd) / 2;
           const activeSections = sections.filter(s => s.md_top <= midMd && s.md_bottom >= midMd);
           if (activeSections.length === 0) return null;
           const innermost = activeSections.reduce((prev, curr) => prev.od < curr.od ? prev : curr);
           const rInner = (innermost.id_hole / 2) * scaleX;
           const sliceTopY = paddingY + (startMd * scaleY);
           const sliceHeight = (endMd - startMd) * scaleY;

           return (
             <rect key={`void-${i}`} x={centerX - rInner} y={sliceTopY} width={rInner * 2} height={sliceHeight} fill="#ffffff" />
           );
        })}

        {/* 4. Details (Shoes) */}
        {sections.map(section => {
          if (section.type === SectionType.OPEN_HOLE) return null;
          const bottomY = paddingY + (section.md_bottom * scaleY);
          const rOuter = (section.od / 2) * scaleX;
          const rInner = (section.id_hole / 2) * scaleX; 
          return (
            <g key={`detail-${section.id}`}>
              <path d={`M ${centerX - rOuter} ${bottomY} L ${centerX - rInner} ${bottomY} L ${centerX - rOuter} ${bottomY - 15} Z`} fill="#000" />
              <path d={`M ${centerX + rOuter} ${bottomY} L ${centerX + rInner} ${bottomY} L ${centerX + rOuter} ${bottomY - 15} Z`} fill="#000" />
            </g>
          )
        })}

        {/* 5. Drill String */}
        {(() => {
            let currentDrawDepth = bitDepth;
            const renderedComponents = [];
            const reversedString = [...drillString].reverse();

            for (const comp of reversedString) {
                const compLen = comp.length * (comp.count || 1);
                const bottomDepth = currentDrawDepth;
                const topDepth = currentDrawDepth - compLen;
                
                if (bottomDepth > 0) {
                   const drawTopY = paddingY + (Math.max(0, topDepth) * scaleY);
                   const drawBottomY = paddingY + (bottomDepth * scaleY);
                   const drawHeight = drawBottomY - drawTopY;
                   const rComp = (comp.od / 2) * scaleX;
                   const rID = ((comp.id_pipe || 0) / 2) * scaleX;
                   const tooltipText = `${comp.name}\nOD: ${comp.od}"\nID: ${comp.id_pipe}"\nLength: ${compLen.toFixed(2)}m`;

                   if (comp.type === ComponentType.BIT) {
                      // === PDC BIT RENDERING ===
                      const bitHeightPixels = Math.max(30, comp.length * scaleY);
                      const bitY = paddingY + (bottomDepth * scaleY) - bitHeightPixels;
                      
                      const rBit = rComp; // OD
                      const shankW = rBit * 0.6;
                      const bodyH = bitHeightPixels * 0.6;
                      const faceH = bitHeightPixels * 0.4;

                      renderedComponents.push(
                        <g key={`bit-${comp.id}`}>
                           <title>{tooltipText}</title>
                           {/* Shank */}
                           <rect 
                             x={centerX - shankW} 
                             y={bitY} 
                             width={shankW * 2} 
                             height={bodyH * 0.5} 
                             fill="url(#steelGradient)" 
                             stroke="#334155"
                           />
                           
                           {/* Gauge / Body */}
                           <rect 
                              x={centerX - rBit}
                              y={bitY + bodyH * 0.5}
                              width={rBit * 2}
                              height={bodyH * 0.5}
                              fill="#334155"
                              stroke="#0f172a"
                           />

                           {/* Bit Face */}
                           <path 
                              d={`
                                M ${centerX - rBit} ${bitY + bodyH} 
                                Q ${centerX - rBit * 0.5} ${bitY + bodyH + faceH} ${centerX} ${bitY + bodyH + faceH} 
                                Q ${centerX + rBit * 0.5} ${bitY + bodyH + faceH} ${centerX + rBit} ${bitY + bodyH}
                                Z
                              `}
                              fill="url(#bitPattern)" 
                              stroke="#0f172a"
                              strokeWidth="1.5"
                           />
                           
                           <line x1={centerX - rBit*0.3} y1={bitY + bodyH} x2={centerX - rBit*0.3} y2={bitY + bodyH + faceH*0.8} stroke="#000" strokeWidth="2" opacity="0.3" />
                           <line x1={centerX + rBit*0.3} y1={bitY + bodyH} x2={centerX + rBit*0.3} y2={bitY + bodyH + faceH*0.8} stroke="#000" strokeWidth="2" opacity="0.3" />
                        </g>
                      );

                   } else if (comp.type === ComponentType.STABILIZER && comp.stabilizer) {
                      // === STABILIZER RENDERING ===
                      const stab = comp.stabilizer;
                      const bladeH = stab.bladeLength * scaleY;
                      const bladeBottomY = drawBottomY - (stab.distFromBottom * scaleY);
                      const bladeTopY = bladeBottomY - bladeH;
                      const rBlade = (stab.bladeOd / 2) * scaleX;
                      const rBody = (comp.od / 2) * scaleX;

                      renderedComponents.push(
                        <g key={comp.id}>
                           <title>{tooltipText}</title>
                           <rect x={centerX - rBody} y={drawTopY} width={rBody * 2} height={drawHeight} fill="#94a3b8" stroke="#475569" />
                           
                           <rect x={centerX - rBlade} y={bladeTopY} width={rBlade - rBody} height={bladeH} fill="url(#stabPattern)" stroke="#1e293b" rx="2" />
                           <rect x={centerX + rBody} y={bladeTopY} width={rBlade - rBody} height={bladeH} fill="url(#stabPattern)" stroke="#1e293b" rx="2" />
                        </g>
                      );
                   } else {
                      // === STANDARD PIPE ===
                      renderedComponents.push(
                        <g key={comp.id}>
                           <title>{tooltipText}</title>
                           <rect x={centerX - rComp} y={drawTopY} width={rComp * 2} height={drawHeight} fill="#94a3b8" stroke="#475569" strokeWidth="1" />
                           {comp.id_pipe > 0 && (
                             <>
                               <line x1={centerX - rID} y1={drawTopY} x2={centerX - rID} y2={drawBottomY} stroke="#cbd5e1" strokeWidth="1" strokeDasharray="4 2" />
                               <line x1={centerX + rID} y1={drawTopY} x2={centerX + rID} y2={drawBottomY} stroke="#cbd5e1" strokeWidth="1" strokeDasharray="4 2" />
                             </>
                           )}
                        </g>
                      );
                   }
                }
                currentDrawDepth = topDepth;
            }
            return renderedComponents;
        })()}

        {/* Depth Axis */}
        <line x1={40} y1={paddingY} x2={40} y2={height-paddingY} stroke="#64748b" strokeWidth="1" />
        {Array.from({length: Math.ceil(maxMD / 500) + 1}, (_, i) => i * 500).map(depth => {
           if(depth > maxMD) return null;
           const y = paddingY + (depth * scaleY);
           return (
              <g key={depth}>
                 <line x1={35} y1={y} x2={45} y2={y} stroke="#64748b" strokeWidth="1" />
                 <text x={30} y={y + 3} fontSize="10" fill="#64748b" textAnchor="end" fontFamily="monospace">{depth}</text>
              </g>
           )
        })}
      </svg>
    </div>
  );
};

export const WellSchematic = React.memo(WellSchematicComponent);
