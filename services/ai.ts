
import { GoogleGenAI, FunctionDeclaration, Type } from "@google/genai";
import { SimulationConfig, StringComponent, ComponentType, TelemetryPoint } from "../types";
import { AVAILABLE_COMPONENTS } from "../constants";

const updateDrillStringFunction: FunctionDeclaration = {
  name: 'updateDrillString',
  description: 'Updates the drill string configuration (BHA) with a new list of components. Call this function immediately when the user asks to suggest, design, optimize, or change the drill string.',
  parameters: {
    type: Type.OBJECT,
    properties: {
      components: {
        type: Type.ARRAY,
        description: "List of components from top to bottom. Prefer using standard components from the catalog where possible.",
        items: {
          type: Type.OBJECT,
          properties: {
            type: { type: Type.STRING, description: "Component type enum: drill_pipe, hwdp, drill_collar, stabilizer, jar, sub, bit" },
            name: { type: Type.STRING, description: "Descriptive name (e.g. 5in Drill Pipe)" },
            od: { type: Type.NUMBER, description: "Outer Diameter in inches" },
            id_pipe: { type: Type.NUMBER, description: "Inner Diameter in inches" },
            length: { type: Type.NUMBER, description: "Length in meters per item" },
            weight: { type: Type.NUMBER, description: "Weight in kg/m" },
            count: { type: Type.NUMBER, description: "Number of items in this stack section" }
          },
          required: ["type", "name", "od", "length", "weight", "count"]
        }
      }
    },
    required: ["components"]
  }
};

export class DrillCopilot {
  private ai: GoogleGenAI;
  private modelName = 'gemini-2.5-flash';

  constructor() {
    this.ai = new GoogleGenAI({ apiKey: process.env.API_KEY });
  }

  async sendMessage(
    message: string, 
    config: SimulationConfig, 
    telemetry: TelemetryPoint[],
    onUpdateConfig: (components: StringComponent[]) => void
  ): Promise<string> {
    
    // Build Catalog String for Context
    const catalogStr = AVAILABLE_COMPONENTS.map(c => 
      `- Name: "${c.name}", Type: ${c.type}, OD: ${c.od}", ID: ${c.id_pipe}", Length: ${c.length}m, Weight: ${c.weight}kg/m`
    ).join('\n');

    // Construct Context
    const context = `
      CURRENT WELL CONTEXT:
      - Total Depth: ${config.wellPath[config.wellPath.length-1].md}m
      - Max Inclination: ${Math.max(...config.wellPath.map(p => p.inclination))} deg
      
      CURRENT DRILL STRING (${config.drillString.length} sections):
      ${config.drillString.map(c => `- ${c.count}x ${c.name} (OD: ${c.od}", Type: ${c.type})`).join('\n')}

      RECENT TELEMETRY:
      ${telemetry.length > 0 ? JSON.stringify(telemetry[telemetry.length-1]) : "Simulation stopped."}

      USER REQUEST: ${message}
    `;

    const systemInstruction = `You are "DrillBit", an expert Drilling Engineering AI.
    
    YOUR GOAL:
    Assist with BHA (Bottom Hole Assembly) design, drill string optimization, and drilling parameter analysis.

    AVAILABLE COMPONENT CATALOG:
    Use these exact specifications when designing or suggesting strings:
    ${catalogStr}

    RULES FOR BHA DESIGN:
    1. ALWAYS start with a BIT at the bottom.
    2. Place STABILIZERS near the bit for directional control.
    3. Use DRILL COLLARS (DC) above the bit/stabilizers to provide Weight on Bit (WOB).
    4. Use HEAVY WEIGHT DRILL PIPE (HWDP) as a transition zone.
    5. FIll the rest to surface with DRILL PIPE.
    6. When the user asks for a design, you MUST use the 'updateDrillString' tool to apply it. Do not just talk about it.
    7. For 'Drill Pipe' and 'HWDP', use the 'count' property to define total length (e.g., count: 50). For BHA items (Bit, Stab, MWD), count is usually 1.

    Be helpful, concise, and engineering-focused.
    `;

    try {
      const response = await this.ai.models.generateContent({
        model: this.modelName,
        contents: context,
        config: {
          systemInstruction: systemInstruction,
          tools: [{ functionDeclarations: [updateDrillStringFunction] }],
        }
      });

      const toolCalls = response.functionCalls;
      let replyText = response.text || "";

      if (toolCalls && toolCalls.length > 0) {
        for (const call of toolCalls) {
          if (call.name === 'updateDrillString') {
            const args = call.args as any;
            // Validate and hydrate the components
            if (args.components && Array.isArray(args.components)) {
                const newComponents: StringComponent[] = args.components.map((c: any) => ({
                  ...c,
                  id: Math.random().toString(36).substr(2, 9), // Generate new IDs
                  id_pipe: c.id_pipe || (c.od * 0.85), // Fallback ID if missing
                  count: c.count || 1,
                  stabilizer: c.type === 'stabilizer' ? { bladeOd: c.od * 1.2, bladeLength: 1.0, distFromBottom: 0.5 } : undefined 
                }));
                
                onUpdateConfig(newComponents);
                replyText += "\n\n[System] I have updated the drill string configuration based on your request.";
            }
          }
        }
      }

      if (!replyText && toolCalls && toolCalls.length > 0) {
         replyText = "I've updated the drill string configuration for you.";
      }

      return replyText;
    } catch (error) {
      console.error("AI Error:", error);
      return "I encountered an error processing your request. Please check your API key configuration.";
    }
  }
}
