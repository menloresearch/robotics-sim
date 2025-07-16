
export const G1_CFG = {
    dof: 43,
    kp: new Float64Array(43).fill(50).fill(150, 12, 15),
    kd: new Float64Array(43).fill(2).fill(4, 12, 15),
    target_dq: new Float64Array(43).fill(0),
}


const clamp = (value, min, max) => {
  return Math.max(min, Math.min(value, max));
};


export function pd_control(target_q, q, kp, target_dq, dq, kd) {
  if (target_q.length != q.length) throw Error("target_q.length != q.length")
  if (target_dq.length != dq.length) throw Error("target_dq.length != dq.length")

  // return (target_q - q) * kp + (target_dq - dq) * kd
  let out = new Float64Array(target_q.length);
  for (let i = 0; i < target_q.length; i++) {
    const tq = target_q[i];
    const tdq = target_dq[i];
    out[i] = (tq - q[i]) * kp[i] + (tdq - dq[i]) * kd[i];
  }
  return out.map(num => clamp(num, -100, 100));
}


export class ActionQuee {
  constructor(pos) {
    this.queue = []
    this.last_pos = pos
  }

  clamp_action(prev, tar) {
    const max_rad_d = Math.PI / 8
    return tar.map((v, i) => {
      const d = Math.min(Math.max(v - prev[i], -max_rad_d), max_rad_d)
      return prev[i] + d
    })
  }

  set_next(pos, steps) {
    let prev = this.queue.length > 0 ? this.queue[this.queue.length - 1] : this.last_pos
    pos = this.clamp_action(prev, pos)
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


export function g1_next_action(user_pose) {
    let tar_q = new Float64Array(G1_CFG.dof).fill(0);  // target joint position
    if (!user_pose) return tar_q

    if ("rightShoulder-leftShoulder-leftElbow" in user_pose) {
        let roll = user_pose['rightShoulder-leftShoulder-leftElbow'].roll;
        let pitch = user_pose['rightShoulder-leftShoulder-leftElbow'].pitch;
        // console.log('roll', roll, 'pitch', pitch)
        tar_q[16] = roll // left_shoulder_roll_joint
        tar_q[15] = pitch - Math.PI / 2 // left_shoulder_pitch
    }
    
    if ("leftShoulder-leftElbow-leftWrist" in user_pose) {
        let pitch = user_pose['leftShoulder-leftElbow-leftWrist'].pitch;
        let yaw = user_pose['leftShoulder-leftElbow-leftWrist'].yaw;
        // console.log('pitch', pitch, 'yaw', yaw)
        tar_q[18] = pitch - Math.PI / 2 // left_elbow_joint
        tar_q[17] = Math.min(Math.max(yaw * 1.5, -Math.PI / 2), Math.PI / 2) // left_shoulder_yaw_joint
    }
    
    if ("left-thumb-1" in user_pose) {
        let pitch = user_pose['left-thumb-1'].pitch;
        tar_q[23] = (Math.PI - Math.abs(pitch));
        // console.log('pitch-1', pitch)
    }
    
    if ("left-thumb-2" in user_pose) {
        let pitch = user_pose['left-thumb-2'].pitch;
        tar_q[24] = (Math.PI - Math.abs(pitch));
        // console.log('pitch-2', pitch)
    }
    
    if ("left-index-0" in user_pose) {
        let pitch = user_pose['left-index-0'].pitch;
        tar_q[27] = -pitch
        // console.log('pitch-1', pitch)
    }
    
    if ("left-index-1" in user_pose) {
        let pitch = user_pose['left-index-1'].pitch;
        tar_q[28] = -pitch
        // console.log('pitch-2', pitch)
    }
    
    if ("left-middle-0" in user_pose) {
        let pitch = user_pose['left-middle-0'].pitch;
        tar_q[25] = -pitch
        // console.log('pitch-1', pitch)
    }
    
    if ("left-middle-1" in user_pose) {
        let pitch = user_pose['left-middle-1'].pitch;
        tar_q[26] = -pitch
        // console.log('pitch-2', pitch)
    }

    if ("leftShoulder-rightShoulder-rightElbow" in user_pose) {
        let roll = user_pose['leftShoulder-rightShoulder-rightElbow'].roll;
        let pitch = user_pose['leftShoulder-rightShoulder-rightElbow'].pitch;
        // console.log('roll', roll, 'pitch', pitch)
        tar_q[30] = roll // left_shoulder_roll_joint
        tar_q[29] = -(pitch - Math.PI / 2) // left_shoulder_pitch
    }

    if ("rightShoulder-rightElbow-rightWrist" in user_pose) {
        let pitch = user_pose['rightShoulder-rightElbow-rightWrist'].pitch;
        let yaw = user_pose['rightShoulder-rightElbow-rightWrist'].yaw;
        yaw += Math.PI / 2;
        // console.log('pitch', pitch, 'yaw', yaw)
        tar_q[32] = pitch - Math.PI / 2 // left_elbow_joint
        tar_q[31] = Math.min(Math.max(yaw * 1.5, -Math.PI / 2), Math.PI / 2) // left_shoulder_yaw_joint
    }

    if ("right-thumb-1" in user_pose) {
        let pitch = user_pose['right-thumb-1'].pitch;
        tar_q[37] = -(Math.PI - Math.abs(pitch));
        // console.log('pitch-1', pitch)
    }
    
    if ("right-thumb-2" in user_pose) {
        let pitch = user_pose['right-thumb-2'].pitch;
        tar_q[38] = -(Math.PI - Math.abs(pitch));
        // console.log('pitch-2', pitch)
    }
    
    if ("right-index-0" in user_pose) {
        let pitch = user_pose['right-index-0'].pitch;
        tar_q[39] = pitch > 0.5 ? 3 : -3
        // console.log('pitch-0', pitch)
    }
    
    if ("right-index-1" in user_pose) {
        let pitch = user_pose['right-index-1'].pitch;
        tar_q[40] = pitch > 0.5 ? 3 : -3
        // tar_q[39] = pitch
        // console.log('pitch-1', pitch)
    }
    
    if ("right-middle-0" in user_pose) {
        // <0: open, 0: close
        let pitch = user_pose['right-middle-1'].pitch;
        // tar_q[41] = this.simulation.qpos[41] > 0.5 ? 0 : 1.7
        // tar_q[41] = 1.7
        tar_q[41] = pitch > 0.5 ? 3 : -3
        // tar_q[41] = (-pitch + Math.PI / 2)
        // console.log('pitch-0', pitch)
    }
    
    if ("right-middle-1" in user_pose) {
        // <0: open, 0: open
        let pitch = user_pose['right-middle-1'].pitch;
        // tar_q[42] = this.simulation.qpos[42] > 0.5 ? 0 : 1.7
        // tar_q[42] = 1.7
        tar_q[42] = pitch > 0.5 ? 3 : -3
        // console.log('pitch-1', pitch)
    }

    return tar_q;
}