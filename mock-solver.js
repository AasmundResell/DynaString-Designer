import express from 'express';
import { WebSocketServer } from 'ws';
import http from 'http';
import cors from 'cors';

const app = express();
const port = 8080;

app.use(cors());
app.use(express.json());

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

let isRunning = false;
let simulationState = {
    t: 0,
    v_top: 0,
    omega_top: 0,
    depth: 0,
    rpm: 0
};

// Mock Simulation Loop
setInterval(() => {
    if (isRunning) {
        simulationState.t += 0.1;
        simulationState.depth += simulationState.v_top * 0.1;
        
        // Generate some fake sine wave data for the string
        const points = [];
        for (let i = 0; i < 100; i++) {
            const s = i * 10;
            points.push({
                s: s,
                x: Math.sin(s * 0.1 + simulationState.t) * 0.5,
                y: Math.cos(s * 0.1 + simulationState.t) * 0.5,
                z: s + simulationState.depth,
                tension: 1000 + Math.sin(s * 0.05) * 500
            });
        }

        const broadcastData = JSON.stringify({
            type: 'data',
            time: simulationState.t,
            depth: simulationState.depth,
            rpm: simulationState.rpm,
            points: points
        });

        wss.clients.forEach(client => {
            if (client.readyState === 1) {
                client.send(broadcastData);
            }
        });
    }
}, 100); // 10Hz update

app.post('/start', (req, res) => {
    console.log('Starting simulation', req.body);
    simulationState = { ...simulationState, ...req.body };
    isRunning = true;
    res.json({ status: 'started' });
});

app.post('/stop', (req, res) => {
    console.log('Stopping simulation');
    isRunning = false;
    res.json({ status: 'stopped' });
});

app.post('/update', (req, res) => {
    console.log('Updating simulation', req.body);
    simulationState = { ...simulationState, ...req.body };
    res.json({ status: 'updated' });
});

server.listen(port, () => {
    console.log(`Mock Solver running at http://localhost:${port}`);
});
