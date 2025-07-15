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
import {
  computeJointEulers,
  PoseEst,
} from "./poseUtils.js"
import load_mujoco from "../dist/mujoco-wasm.js";

const clamp = (value, min, max) => {
  return Math.max(min, Math.min(value, max));
};

function pd_control(target_q, q, kp, target_dq, dq, kd) {
  if (target_q.length != q.length) throw Error("target_q.length != q.length")
  if (target_dq.length != dq.length) throw Error("target_dq.length != dq.length")

  // return (target_q - q) * kp + (target_dq - dq) * kd
  let out = new Float64Array(target_q.length);
  for (let i = 0; i < target_q.length; i++) {
    const tq = target_q[i];
    const tdq = target_dq[i];
    out[i] = (tq - q[i]) * kp[i] + (tdq - dq[i]) * kd[i];
  }
  // console.debug(out);
  // console.debug(Math.max(...out), Math.min(...out))
  return out.map(num => clamp(num, -100, 100));
  // debugger
  return out
}

const g1_dof = 43
let g1_kp = new Float64Array(g1_dof).fill(50).fill(100, 12, 15);
let g1_kd = new Float64Array(g1_dof).fill(2).fill(3, 12, 15);;
// let g1_kp = new Float64Array(g1_dof).fill(0)
// let g1_kd = new Float64Array(g1_dof).fill(0)
const g1_target_dq = new Float64Array(g1_dof).fill(0);


class ActionQuee {
  constructor(pos) {
    this.queue = []
    this.last_pos = pos
  }

  set_next(pos, steps) {
    let prev = this.queue.length > 0 ? this.queue[this.queue.length - 1] : this.last_pos
    const delta = pos.map((v, i) => {
      return (v - prev[i]) / steps
    })
    for (let i = 1; i < steps + 1; i++) {
      let intrep = new Float64Array(prev)
      intrep = intrep.map((v, j) => v + delta[j] * i)
      this.queue.push(intrep)
    }
    // debugger
  }

  next_action() {
    if (this.queue.length > 0)
      this.last_pos = this.queue.shift()
    return this.last_pos
  }
}

// Load the MuJoCo Module
const mujoco = await load_mujoco();
const pose_est = new PoseEst();

// Set up Emscripten's Virtual File System
var initialScene = "arm26.xml";
// var initialScene = "g1_29dof_with_hand_rev_1_0.xml";

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

    this._debug_step = 0
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  control_step() {
    let timestep = this.model.getOptions().timestep;
    
    if (this.params.scene === 'g1_29dof_with_hand_rev_1_0.xml' && (this._debug_step > 100) && (this._debug_step % 1 == 0)) {
          
      let tar_q = new Float64Array(g1_dof).fill(0);
      let zero = new Float64Array(g1_dof).fill(0);

      const default_p = pd_control(
        zero,
        this.simulation.qpos,
        // this.simulation.qpos.subarray(7),
        g1_kp,
        g1_target_dq,
        this.simulation.qvel,
        g1_kd,
      )

            
      if (this.action_que === undefined) {
        this.action_que = new ActionQuee(new Float64Array(this.simulation.qpos))
        this.action_que.set_next(tar_q, 100);
      } else {
        if (this.action_que.queue.length === 0) {
          if (pose_est.cur_joint_pos && "rightShoulder-leftShoulder-leftElbow" in pose_est.cur_joint_pos) {
            let roll = pose_est.cur_joint_pos['rightShoulder-leftShoulder-leftElbow'].roll;
            let pitch = pose_est.cur_joint_pos['rightShoulder-leftShoulder-leftElbow'].pitch;
            // console.log('roll', roll, 'pitch', pitch)
            tar_q[16] = roll // left_shoulder_roll_joint
            tar_q[15] = pitch - Math.PI / 2 // left_shoulder_pitch
          }
          
          if (pose_est.cur_joint_pos && "leftShoulder-leftElbow-leftWrist" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['leftShoulder-leftElbow-leftWrist'].pitch;
            let yaw = pose_est.cur_joint_pos['leftShoulder-leftElbow-leftWrist'].yaw;
            // console.log('pitch', pitch, 'yaw', yaw)
            tar_q[18] = pitch - Math.PI / 2 // left_elbow_joint
            tar_q[17] = yaw // left_shoulder_yaw_joint
          }
          
          if (pose_est.cur_joint_pos && "left-thumb-1" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['left-thumb-1'].pitch;
            tar_q[23] = (Math.PI - Math.abs(pitch));
            // console.log('pitch-1', pitch)
          }
          
          if (pose_est.cur_joint_pos && "left-thumb-2" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['left-thumb-2'].pitch;
            tar_q[24] = (Math.PI - Math.abs(pitch));
            // console.log('pitch-2', pitch)
          }
          
          if (pose_est.cur_joint_pos && "left-index-0" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['left-index-0'].pitch;
            tar_q[27] = -pitch
            console.log('pitch-1', pitch)
          }
          
          if (pose_est.cur_joint_pos && "left-index-1" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['left-index-1'].pitch;
            tar_q[28] = -pitch
            console.log('pitch-2', pitch)
          }
          
          if (pose_est.cur_joint_pos && "left-middle-0" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['left-middle-0'].pitch;
            tar_q[25] = -pitch
            console.log('pitch-1', pitch)
          }
          
          if (pose_est.cur_joint_pos && "left-middle-1" in pose_est.cur_joint_pos) {
            let pitch = pose_est.cur_joint_pos['left-middle-1'].pitch;
            tar_q[26] = -pitch
            console.log('pitch-2', pitch)
          }
         
          this.action_que.set_next(tar_q, 100);
        }
      }
      
      const tau = pd_control(
        this.action_que.next_action(),
        this.simulation.qpos,
        g1_kp,
        g1_target_dq,
        this.simulation.qvel,
        g1_kd,
      )
      for (let i = 0; i < this.simulation.ctrl.length; i++) {
        this.simulation.ctrl[i] = tau[i];
        // this.simulation.ctrl[i] = tmp[i];
      }
      // this.simulation.ctrl[3] = tau[3];
      this.simulation.ctrl[12] = default_p[12];
      this.simulation.ctrl[13] = default_p[13];
      this.simulation.ctrl[14] = default_p[14];
      
      // this.simulation.ctrl[15] = tau[15]; // left shoulder pitch
      // this.simulation.ctrl[16] = tau[16]; // left shoulder roll
      // this.simulation.ctrl[18] = tau[18]; // left_elbow
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

  render(timeMS) {
    this.controls.update();
    this._debug_step++
    
    if (!this.params["paused"]) {
      if (timeMS - this.mujoco_time > 35.0) {
        this.mujoco_time = timeMS;
      }
      while (this.mujoco_time < timeMS) {
        this.control_step()
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
