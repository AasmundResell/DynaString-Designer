import { SolverConfigStatic, SolverFrameData } from '../types';

const API_URL = 'http://localhost:8080';
const WS_URL = 'ws://localhost:8080';

export class SolverService {
  private ws: WebSocket | null = null;
  private onDataCallback: ((data: SolverFrameData) => void) | null = null;

  async startSimulation(config: Partial<SolverConfigStatic>): Promise<void> {
    const response = await fetch(`${API_URL}/start`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });
    if (!response.ok) throw new Error('Failed to start simulation');
  }

  async stopSimulation(): Promise<void> {
    const response = await fetch(`${API_URL}/stop`, {
      method: 'POST',
    });
    if (!response.ok) throw new Error('Failed to stop simulation');
  }

  async updateControl(params: { v_top_input?: number; omega_top_input?: number }): Promise<void> {
    const response = await fetch(`${API_URL}/update`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(params),
    });
    if (!response.ok) throw new Error('Failed to update simulation');
  }

  connectDataStream(onData: (data: SolverFrameData) => void): void {
    this.onDataCallback = onData;
    
    if (this.ws) {
      this.ws.close();
    }

    this.ws = new WebSocket(WS_URL);

    this.ws.onopen = () => {
      console.log('Connected to Solver Stream');
    };

    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === 'data' && this.onDataCallback) {
          this.onDataCallback(message as SolverFrameData);
        }
      } catch (e) {
        console.error('Failed to parse solver message', e);
      }
    };

    this.ws.onclose = () => {
      console.log('Solver Stream Disconnected');
    };
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

export const solverService = new SolverService();
