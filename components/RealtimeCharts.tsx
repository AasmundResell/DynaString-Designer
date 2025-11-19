import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, ReferenceLine, AreaChart, Area } from 'recharts';
import { TelemetryPoint } from '../types';

interface Props {
  data: TelemetryPoint[];
  targetWob: number;
  targetRpm: number;
}

export const RealtimeCharts: React.FC<Props> = ({ data, targetWob, targetRpm }) => {
  // We only show the last 60 points for performance
  const recentData = data.slice(-60);

  return (
    <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 h-full overflow-y-auto p-4 bg-slate-50">
      
      {/* WOB Chart */}
      <div className="bg-white border border-slate-200 p-4 rounded-lg shadow-sm h-64 flex flex-col">
        <div className="flex justify-between items-center mb-2">
           <h4 className="text-slate-600 text-sm font-semibold">Weight on Bit (kN)</h4>
           <div className="text-cyan-600 font-mono text-lg">{recentData[recentData.length-1]?.wob.toFixed(1)}</div>
        </div>
        <div className="flex-1">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={recentData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
              <XAxis dataKey="timestamp" hide />
              <YAxis domain={[0, 'auto']} stroke="#94a3b8" fontSize={12} />
              <Tooltip 
                contentStyle={{ backgroundColor: '#ffffff', border: '1px solid #cbd5e1', borderRadius: '6px', boxShadow: '0 2px 4px rgba(0,0,0,0.05)' }}
                itemStyle={{ color: '#0891b2' }}
                labelStyle={{ display: 'none' }}
                formatter={(val: number) => [val.toFixed(1), 'kN']}
              />
              <ReferenceLine y={targetWob} stroke="#cbd5e1" strokeDasharray="3 3" label={{ value: 'Target', fill: '#94a3b8', fontSize: 10 }} />
              <Area type="monotone" dataKey="wob" stroke="#06b6d4" fill="#06b6d4" fillOpacity={0.1} isAnimationActive={false} />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </div>

      {/* RPM Chart */}
      <div className="bg-white border border-slate-200 p-4 rounded-lg shadow-sm h-64 flex flex-col">
        <div className="flex justify-between items-center mb-2">
           <h4 className="text-slate-600 text-sm font-semibold">RPM (Surf vs Bit)</h4>
           <div className="text-emerald-600 font-mono text-lg">{recentData[recentData.length-1]?.rpmBit.toFixed(0)}</div>
        </div>
        <div className="flex-1">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={recentData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
              <XAxis dataKey="timestamp" hide />
              <YAxis domain={[0, 'auto']} stroke="#94a3b8" fontSize={12} />
              <Tooltip 
                contentStyle={{ backgroundColor: '#ffffff', border: '1px solid #cbd5e1', borderRadius: '6px', boxShadow: '0 2px 4px rgba(0,0,0,0.05)' }}
                labelStyle={{ display: 'none' }}
              />
              <Line type="monotone" dataKey="rpmSurf" stroke="#cbd5e1" dot={false} strokeWidth={1} isAnimationActive={false} name="Surf" />
              <Line type="monotone" dataKey="rpmBit" stroke="#10b981" dot={false} strokeWidth={2} isAnimationActive={false} name="Bit" />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>

       {/* Torque Chart */}
       <div className="bg-white border border-slate-200 p-4 rounded-lg shadow-sm h-64 flex flex-col">
        <div className="flex justify-between items-center mb-2">
           <h4 className="text-slate-600 text-sm font-semibold">Torque on Bit (kNm)</h4>
           <div className="text-orange-500 font-mono text-lg">{recentData[recentData.length-1]?.tob.toFixed(2)}</div>
        </div>
        <div className="flex-1">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={recentData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
              <XAxis dataKey="timestamp" hide />
              <YAxis domain={[0, 'auto']} stroke="#94a3b8" fontSize={12} />
              <Tooltip 
                contentStyle={{ backgroundColor: '#ffffff', border: '1px solid #cbd5e1', borderRadius: '6px', boxShadow: '0 2px 4px rgba(0,0,0,0.05)' }}
                labelStyle={{ display: 'none' }}
              />
              <Line type="monotone" dataKey="tob" stroke="#f97316" dot={false} strokeWidth={2} isAnimationActive={false} />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>

      {/* Hookload Chart */}
      <div className="bg-white border border-slate-200 p-4 rounded-lg shadow-sm h-64 flex flex-col">
        <div className="flex justify-between items-center mb-2">
           <h4 className="text-slate-600 text-sm font-semibold">Hook Load (kN)</h4>
           <div className="text-purple-600 font-mono text-lg">{recentData[recentData.length-1]?.hookLoad.toFixed(0)}</div>
        </div>
        <div className="flex-1">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={recentData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
              <XAxis dataKey="timestamp" hide />
              <YAxis domain={['dataMin - 50', 'dataMax + 50']} stroke="#94a3b8" fontSize={12} />
              <Tooltip 
                contentStyle={{ backgroundColor: '#ffffff', border: '1px solid #cbd5e1', borderRadius: '6px', boxShadow: '0 2px 4px rgba(0,0,0,0.05)' }}
                labelStyle={{ display: 'none' }}
              />
              <Area type="monotone" dataKey="hookLoad" stroke="#a855f7" fill="#a855f7" fillOpacity={0.1} isAnimationActive={false} />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </div>

    </div>
  );
};