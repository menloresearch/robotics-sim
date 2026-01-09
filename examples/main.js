import * as THREE from "three";
import { GUI } from "../node_modules/three/examples/jsm/libs/lil-gui.module.min.js";
import { OrbitControls } from "../node_modules/three/examples/jsm/controls/OrbitControls.js";
import { DragStateManager } from "./utils/DragStateManager.js";
import {
  setupGUI,
  downloadExampleScenesFolder,
  loadSceneFromURL,
  getPosition,
  getQuaternion,
  toMujocoPos,
  standardNormal,
} from "./mujocoUtils.js";
import { VisualizationController } from "./visualizationController.js";
import load_mujoco from "../dist/mujoco-wasm.js";


// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
// var initialScene = "arm26.xml";
var initialScene = "g1_29dof_with_hand_rev_1_0.xml";

mujoco.FS.mkdir("/working");
mujoco.FS.mount(mujoco.MEMFS, { root: "." }, "/working");
mujoco.FS.writeFile(
  "/working/" + initialScene,
  await (await fetch("./examples/scenes/" + initialScene)).text(),
);

export class MuJoCoDemo {
  
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML
    this.model = new mujoco.Model("/working/" + initialScene);
    this.state = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);

    // Define Random State Variables
    this.params = {
      scene: initialScene,
      paused: false,
      help: false,
      ctrlnoiserate: 0.0,
      ctrlnoisestd: 0.0,
      keyframeNumber: 0,
    };
    this.mujoco_time = 0.0;
    (this.bodies = {}), (this.lights = {});
    this.tmpVec = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement("div");
    document.body.appendChild(this.container);

    this.scene = new THREE.Scene();
    this.scene.name = "scene";

    this.camera = new THREE.PerspectiveCamera(
      45,
      window.innerWidth / window.innerHeight,
      0.001,
      100,
    );
    this.camera.name = "PerspectiveCamera";
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1);
    this.ambientLight.name = "AmbientLight";
    this.scene.add(this.ambientLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    this.renderer.setAnimationLoop(this.render.bind(this));

    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener("resize", this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(
      this.scene,
      this.renderer,
      this.camera,
      this.container.parentElement,
      this.controls,
    );

    this._debug_step = 0;

    // TWIST2 Visualization Controller
    this.twist2Controller = null;
    this.twist2Ready = false;
    this.visualizationMode = true;
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    // Initialize TWIST2 controller if G1 model is loaded
    console.log('[DEBUG] Current scene:', this.params.scene);
    if (this.params.scene === 'g1_29dof_with_hand_rev_1_0.xml') {
      console.log('[DEBUG] Initializing TWIST2 for G1...');
      await this.initTWIST2();
    } else {
      console.log('[DEBUG] Skipping TWIST2 - not G1 scene');
    }

    this.gui = new GUI();
    setupGUI(this);
  }

  async initTWIST2() {
    // Change this IP to where your WebSocket bridge is running
    // If on host machine: use host IP
    // If on same VM: use localhost
    const WS_URL = "ws://10.210.0.51:8765"; // Update this IP!

    console.log('[TWIST2] Visualization mode: Browser mirrors Python simulation');
    console.log('[TWIST2] Make sure to run: python server_low_level_g1_sim.py');

    this.twist2Controller = new VisualizationController(this.simulation, WS_URL);
    this.visualizationMode = true;
    this.twist2Ready = await this.twist2Controller.initialize();

    if (this.twist2Ready) {
      console.log('[TWIST2] Controller ready!');
      this.updateTWIST2Status('policy', 'Ready', 'lime');
    } else {
      console.error('[TWIST2] Failed to initialize controller');
      this.updateTWIST2Status('policy', 'Failed', 'red');
    }
  }

  updateTWIST2Status(type, text, color = 'white') {
    const statusMap = {
      'ws': 'twist2-ws-status',
      'policy': 'twist2-policy-status',
      'redis': 'twist2-redis-status'
    };

    const elementId = statusMap[type];
    if (elementId) {
      const element = document.getElementById(elementId);
      if (element) {
        const span = element.querySelector('span');
        if (span) {
          span.textContent = text;
          span.style.color = color;
        }
      }
    }
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  async control_step() {
    let timestep = this.model.getOptions().timestep;

    // Visualization mode for G1: Just mirror Python simulation
    if (this.params.scene === 'g1_29dof_with_hand_rev_1_0.xml' && this.twist2Controller && this.twist2Ready) {
      // Update pose from Python simulation (no physics)
      this.twist2Controller.step();

      // Update status panel
      if (this.twist2Controller.ws && this.twist2Controller.ws.readyState === WebSocket.OPEN) {
        this.updateTWIST2Status('ws', 'Connected', 'lime');
        if (this.twist2Controller.simQpos) {
          this.updateTWIST2Status('policy', 'Mirroring Python', 'lime');
          this.updateTWIST2Status('redis', 'Receiving State', 'lime');
        } else {
          this.updateTWIST2Status('redis', 'Waiting for Python...', 'yellow');
        }
      } else {
        this.updateTWIST2Status('ws', 'Connecting...', 'yellow');
      }

      // Update visualization without physics
      this.simulation.forward();
      this.mujoco_time += timestep * 1000.0;
      return;
    }

    // Clear old perturbations, apply new ones.
    for (let i = 0; i < this.simulation.qfrc_applied.length; i++) {
      this.simulation.qfrc_applied[i] = 0.0;
    }

    let dragged = this.dragStateManager.physicsObject;
    if (dragged && dragged.bodyID) {
      for (let b = 0; b < this.model.nbody; b++) {
        if (this.bodies[b]) {
          getPosition(this.simulation.xpos, b, this.bodies[b].position);
          getQuaternion(
            this.simulation.xquat,
            b,
            this.bodies[b].quaternion,
          );
          this.bodies[b].updateWorldMatrix();
        }
      }
      let bodyID = dragged.bodyID;
      this.dragStateManager.update(); // Update the world-space force origin
      let force = toMujocoPos(
        this.dragStateManager.currentWorld
          .clone()
          .sub(this.dragStateManager.worldHit)
          .multiplyScalar(this.model.body_mass[bodyID] * 250),
      );
      let point = toMujocoPos(this.dragStateManager.worldHit.clone());
      this.simulation.applyForce(
        force.x,
        force.y,
        force.z,
        0,
        0,
        0,
        point.x,
        point.y,
        point.z,
        bodyID,
      );

      // TODO: Apply pose perturbations (mocap bodies only).
    }

    this.simulation.step();

    this.mujoco_time += timestep * 1000.0;
  }

  async render(timeMS) {
    this.controls.update();
    this._debug_step++

    if (!this.params["paused"]) {
      if (timeMS - this.mujoco_time > 35.0) {
        this.mujoco_time = timeMS;
      }
      while (this.mujoco_time < timeMS) {
        await this.control_step()
      }
    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition(this.simulation.xpos, b, this.tmpVec, false); // Get raw coordinate from MuJoCo
        getQuaternion(this.simulation.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(
          this.dragStateManager.currentWorld
            .clone()
            .sub(this.dragStateManager.worldHit)
            .multiplyScalar(0.3),
        );
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos = this.simulation.mocap_pos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos = this.simulation.qpos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;

          //// Save the original root body position
          //let x  = pos[addr + 0], y  = pos[addr + 1], z  = pos[addr + 2];
          //let xq = pos[addr + 3], yq = pos[addr + 4], zq = pos[addr + 5], wq = pos[addr + 6];

          //// Clear old perturbations, apply new ones.
          //for (let i = 0; i < this.simulation.qfrc_applied().length; i++) { this.simulation.qfrc_applied()[i] = 0.0; }
          //for (let bi = 0; bi < this.model.nbody(); bi++) {
          //  if (this.bodies[b]) {
          //    getPosition  (this.simulation.xpos (), bi, this.bodies[bi].position);
          //    getQuaternion(this.simulation.xquat(), bi, this.bodies[bi].quaternion);
          //    this.bodies[bi].updateWorldMatrix();
          //  }
          //}
          ////dragStateManager.update(); // Update the world-space force origin
          //let force = toMujocoPos(this.dragStateManager.currentWorld.clone()
          //  .sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass()[b] * 0.01));
          //let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          //// This force is dumped into xrfc_applied
          //this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, b);
          //this.simulation.integratePos(this.simulation.qpos(), this.simulation.qfrc_applied(), 1);

          //// Add extra drag to the root body
          //pos[addr + 0] = x  + (pos[addr + 0] - x ) * 0.1;
          //pos[addr + 1] = y  + (pos[addr + 1] - y ) * 0.1;
          //pos[addr + 2] = z  + (pos[addr + 2] - z ) * 0.1;
          //pos[addr + 3] = xq + (pos[addr + 3] - xq) * 0.1;
          //pos[addr + 4] = yq + (pos[addr + 4] - yq) * 0.1;
          //pos[addr + 5] = zq + (pos[addr + 5] - zq) * 0.1;
          //pos[addr + 6] = wq + (pos[addr + 6] - wq) * 0.1;
        }
      }

      this.simulation.forward();
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.simulation.xpos, b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update tendon transforms.
    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (
          let w = startW;
          w < startW + this.simulation.ten_wrapnum[t] - 1;
          w++
        ) {
          let tendonStart = getPosition(
            this.simulation.wrap_xpos,
            w,
            new THREE.Vector3(),
          );
          let tendonEnd = getPosition(
            this.simulation.wrap_xpos,
            w + 1,
            new THREE.Vector3(),
          );
          let tendonAvg = new THREE.Vector3()
            .addVectors(tendonStart, tendonEnd)
            .multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd = tendonEnd.length() > 0.01;

          if (validStart) {
            this.mujocoRoot.spheres.setMatrixAt(
              numWraps,
              mat.compose(
                tendonStart,
                new THREE.Quaternion(),
                new THREE.Vector3(r, r, r),
              ),
            );
          }
          if (validEnd) {
            this.mujocoRoot.spheres.setMatrixAt(
              numWraps + 1,
              mat.compose(
                tendonEnd,
                new THREE.Quaternion(),
                new THREE.Vector3(r, r, r),
              ),
            );
          }
          if (validStart && validEnd) {
            mat.compose(
              tendonAvg,
              new THREE.Quaternion().setFromUnitVectors(
                new THREE.Vector3(0, 1, 0),
                tendonEnd.clone().sub(tendonStart).normalize(),
              ),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r),
            );
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres.count = numWraps > 0 ? numWraps + 1 : 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres.instanceMatrix.needsUpdate = true;
    }

    // Render!
    this.renderer.render(this.scene, this.camera);
  }
}

let demo = new MuJoCoDemo();
await demo.init();
