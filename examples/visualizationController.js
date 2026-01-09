/**
 * Visualization-Only Controller
 * Just mirrors the robot state from Python simulation running locally
 */

export class VisualizationController {
    constructor(simulation, wsURL = 'ws://10.210.0.51:8765') {
        this.simulation = simulation;
        this.wsURL = wsURL;
        this.ws = null;
        this.isReady = false;
        this.simQpos = null;
        this.simQvel = null;

        console.log('[Viz] Visualization Controller created');
    }

    async initialize() {
        console.log('[Viz] Initializing visualization-only mode...');
        console.log('[Viz] This mode just displays the Python simulation state');
        this.connectWebSocket();
        return true;
    }

    connectWebSocket() {
        console.log(`[Viz] Connecting to WebSocket at ${this.wsURL}...`);

        try {
            this.ws = new WebSocket(this.wsURL);
        } catch (error) {
            console.error('[Viz] Failed to create WebSocket:', error);
            return;
        }

        this.ws.onopen = () => {
            console.log('[Viz] ✓ WebSocket connected');
            console.log('[Viz] Waiting for Python simulation state...');
            this.isReady = true;
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);

                // Receive full qpos and qvel from Python simulation
                if (data.sim_qpos_unitree_g1_with_hands) {
                    this.simQpos = new Float64Array(data.sim_qpos_unitree_g1_with_hands);

                    if (!this.hasLoggedQpos) {
                        console.log('[Viz] ✓ Receiving simulation state from Python!');
                        console.log('[Viz] qpos length:', this.simQpos.length);
                        this.hasLoggedQpos = true;
                    }
                }

                if (data.sim_qvel_unitree_g1_with_hands) {
                    this.simQvel = new Float64Array(data.sim_qvel_unitree_g1_with_hands);
                }
            } catch (error) {
                console.error('[Viz] Error parsing WebSocket message:', error);
            }
        };

        this.ws.onerror = (error) => {
            console.error('[Viz] WebSocket error:', error);
            this.isReady = false;
        };

        this.ws.onclose = () => {
            console.log('[Viz] WebSocket disconnected. Reconnecting in 1s...');
            this.isReady = false;
            setTimeout(() => this.connectWebSocket(), 1000);
        };
    }

    step() {
        // If we have simulation state from Python, apply it to browser visualization
        if (this.simQpos && this.simQvel) {
            // Copy qpos and qvel from Python simulation to browser WASM
            for (let i = 0; i < this.simQpos.length && i < this.simulation.qpos.length; i++) {
                this.simulation.qpos[i] = this.simQpos[i];
            }
            for (let i = 0; i < this.simQvel.length && i < this.simulation.qvel.length; i++) {
                this.simulation.qvel[i] = this.simQvel[i];
            }

            // Update visualization without stepping physics
            // (Python is doing the physics, we just render)
        }
        // No torque control needed - Python handles that
    }
}
