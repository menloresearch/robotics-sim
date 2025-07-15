import {medaipipeIK} from './ik.js'

const BODY_JOINTS = {
    leftShoulder: 11,  rightShoulder: 12,
    leftElbow:    13,  rightElbow:    14,
    leftWrist:    15,  rightWrist:    16,
    leftHip:      23,  rightHip:      24,
    leftKnee:     25,  rightKnee:     26,
    leftAnkle:    27,  rightAnkle:    28,
};

const HAND_KEYPOINTS = {
    wrist: 0,

    thumbCMC: 1,
    thumbMCP: 2,
    thumbIP: 3,
    thumbTIP: 4,

    indexFingerMCP: 5,
    indexFingerPIP: 6,
    indexFingerDIP: 7,
    indexFingerTIP: 8,

    middleFingerMCP: 9,
    middleFingerPIP: 10,
    middleFingerDIP: 11,
    middleFingerTIP: 12,

    ringFingerMCP: 13,
    ringFingerPIP: 14,
    ringFingerDIP: 15,
    ringFingerTIP: 16,

    pinkyMCP: 17,
    pinkyPIP: 18,
    pinkyDIP: 19,
    pinkyTIP: 20,
};


const BONES = [
    ['leftShoulder', 'leftElbow'],
    ['leftElbow',    'leftWrist'],
    ['rightShoulder','rightElbow'],
    ['rightElbow',   'rightWrist'],
    ['leftHip',      'leftKnee'],
    ['leftKnee',     'leftAnkle'],
    ['rightHip',     'rightKnee'],
    ['rightKnee',    'rightAnkle'],
    ['leftShoulder', 'rightShoulder'],
    ['rightShoulder', 'leftShoulder'],
];

function empty_obj(value) {
    return value && Object.keys(value).length === 0;
}

class Vec3 {
    constructor(x, y, z) { 
        Object.assign(this, { x, y, z }); 
    }
    sub(b) { 
        return new Vec3(this.x - b.x, this.y - b.y, this.z - b.z); 
    }
    normalize() {
        const len = Math.hypot(this.x, this.y, this.z) || 1;
        return new Vec3(this.x/len, this.y/len, this.z/len);
    }
    cross(b) {
        return new Vec3(
            this.y*b.z - this.z*b.y,
            this.z*b.x - this.x*b.z,
            this.x*b.y - this.y*b.x
        );
    }
    len(){ 
        return Math.hypot(this.x,this.y,this.z); 
    }
    dot(b){
        return this.x*b.x + this.y*b.y + this.z*b.z;
    }
    rad(b) {
        let cos_sim = this.dot(b) / (this.len() * b.len())
        let rad;
        try {
            rad = Math.acos(cos_sim);
        } catch {
            rad = Math.PI / 2;
        }
        return rad
    }
}

function rotationMatrixToEuler(m, options = { unit: 'radian' }) {
  const sy = Math.hypot(m.x.x, m.y.x);
  const singular = sy < 1e-6;
  let roll, pitch, yaw;
  if (!singular) {
    roll  = Math.atan2(m.z.y,  m.z.z);
    pitch = Math.atan2(-m.z.x, sy);
    yaw   = Math.atan2(m.y.x,  m.x.x);
  } else {
    roll  = Math.atan2(-m.y.z, m.y.y);
    pitch = Math.atan2(-m.z.x, sy);
    yaw   = 0;
  }
  const toDeg = rad => rad * (180 / Math.PI);
  const convert = rad => options.unit === 'degree' ? toDeg(rad) : rad;
  return { roll: convert(roll), pitch: convert(pitch), yaw: convert(yaw) };
}

export function computeJointEulers(poseLandmarks, options = { unit: 'radian', visibilityThreshold: 0.5 }) {
  const eulers = {};
  
  for (const [parentName, childName] of BONES) {
    const P = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS[parentName]]));
    const C = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS[childName]]));
    if (P.visibility < options.visibilityThreshold || C.visibility < options.visibilityThreshold) 
        continue;

    const worldUp = new Vec3(0, 1, 0);
    const zAxis = C.sub(P).normalize();
    let xAxis = worldUp.cross(zAxis).normalize();
    // if (Math.hypot(xAxis.x, xAxis.y, xAxis.z) < 1e-3) {
    //   xAxis = new Vec3(1,0,0).cross(zAxis).normalize();
    // }
    const yAxis = zAxis.cross(xAxis).normalize();
    const rotMat = { x: xAxis, y: yAxis, z: zAxis };

    eulers[`${parentName}-${childName}`] = rotationMatrixToEuler(rotMat, options);
  }

  return eulers;
}

function computeRelativeJointAngles(poseLandmarks, options = {
  unit: 'radian',
  visibilityThreshold: 0.5,
}) {
    const triplets = [
        // [proximalJointName, middleJointName, distalJointName]
        ['rightHip','rightKnee','rightAnkle'],
        ['rightShoulder','rightHip','rightKnee'],
        ['leftHip','leftKnee','leftAnkle'],
        ['leftShoulder','leftHip','leftKnee'],
        ['leftShoulder', 'rightShoulder','rightElbow'],
        ['rightShoulder','rightElbow','rightWrist'],
        ['rightShoulder','leftShoulder','leftElbow'],
        ['leftShoulder','leftElbow','leftWrist'],
    ]
    const toDeg = r => r * (180 / Math.PI);
    const conv = r => options.unit === 'degree' ? toDeg(r) : r;

    const results = {};
    const abs_joint_angles = computeJointEulers(poseLandmarks, options);

    for (const [P, M, D] of triplets) {
        const p = poseLandmarks[BODY_JOINTS[P]], m = poseLandmarks[BODY_JOINTS[M]], d = poseLandmarks[BODY_JOINTS[D]];
        if (p.visibility < options.visibilityThreshold ||
            m.visibility < options.visibilityThreshold ||
            d.visibility < options.visibilityThreshold) continue;

        // Get the absolute joint angles for the two bone segments
        const proximaToMiddle = `${P}-${M}`;
        const middleToDistal = `${M}-${D}`;
        
        // Check if we have computed angles for both segments
        if (abs_joint_angles[proximaToMiddle] && abs_joint_angles[middleToDistal]) {
            const parentAngles = abs_joint_angles[proximaToMiddle];
            const childAngles = abs_joint_angles[middleToDistal];
            if ("rightShoulder-leftShoulder-leftElbow".includes(proximaToMiddle) &&
                "rightShoulder-leftShoulder-leftElbow".includes(middleToDistal)
            ) {
                console.log(proximaToMiddle, parentAngles)
                console.log(middleToDistal, childAngles)
            }
            
            // Compute relative angles by subtracting parent from child
            // This gives us the relative rotation needed to go from parent to child orientation
            const relativeAngles = {
                roll: (childAngles.roll - parentAngles.roll) % Math.PI,
                pitch: (childAngles.pitch - parentAngles.pitch),
                yaw: (childAngles.yaw - parentAngles.yaw)
            };
            
            // Store the result with a descriptive key
            results[`${P}-${M}-${D}`] = relativeAngles;
        }
    }

    return results;
}


class KeypointsToAngles {
    constructor() {
        this.startFlag = true;
    }

    vectorFromPoints(P1, P2) {
        return P1.sub(P2)
        // return new Vec3(P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]);
    }

    obtainLShoulderPitchRollAngles(P1, P5, P6, P8) {
        const v_1_5 = this.vectorFromPoints(P1, P5);
        const v_5_1 = this.vectorFromPoints(P5, P1);
        const v_6_5 = this.vectorFromPoints(P6, P5);
        const v_5_6 = this.vectorFromPoints(P5, P6);
        const v_8_1 = this.vectorFromPoints(P1, P8);

        const n_8_1_5 = v_8_1.cross(v_5_1);
        const R_left_torso = n_8_1_5.cross(v_8_1);

        let x = v_5_6.dot(v_8_1) / (v_5_6.len() * v_8_1.len());
        let intermediateAngle;
        try {
            intermediateAngle = Math.acos(x);
        } catch {
            intermediateAngle = Math.PI / 2;
        }

        x = v_8_1.dot(R_left_torso.cross(v_5_6)) / (v_8_1.len() * R_left_torso.cross(v_5_6).len());
        let theta_LSP_module;
        try {
            theta_LSP_module = Math.acos(x);
        } catch {
            theta_LSP_module = 0;
        }

        let LShoulderPitch = intermediateAngle <= Math.PI / 2 ? theta_LSP_module : -theta_LSP_module;

        x = v_5_6.dot(R_left_torso) / (v_5_6.len() * R_left_torso.len());
        let LShoulderRoll;
        try {
            LShoulderRoll = Math.acos(x) - Math.PI / 2;
        } catch {
            LShoulderRoll = 0;
        }

        return { LShoulderPitch, LShoulderRoll };
    }

    obtainRShoulderPitchRollAngles(P1, P2, P3, P8) {
        const v_2_3 = this.vectorFromPoints(P2, P3);
        const v_1_2 = this.vectorFromPoints(P1, P2);
        const v_8_1 = this.vectorFromPoints(P8, P1);

        const n_8_1_2 = v_8_1.cross(v_1_2);
        const R_right_torso = n_8_1_2.cross(v_8_1);

        let x = v_8_1.dot(R_right_torso.cross(v_2_3)) / (v_8_1.len() * R_right_torso.cross(v_2_3).len());
        let theta_RSP_module;
        try {
            theta_RSP_module = Math.acos(x);
        } catch {
            theta_RSP_module = 0;
        }

        x = v_2_3.dot(v_8_1) / (v_2_3.len() * v_8_1.len());
        let intermediateAngle;
        try {
            intermediateAngle = Math.acos(x);
        } catch {
            intermediateAngle = Math.PI / 2;
        }

        let RShoulderPitch = theta_RSP_module;
        // let RShoulderPitch = intermediateAngle <= Math.PI / 2 ? -theta_RSP_module : theta_RSP_module;

        x = v_2_3.dot(R_right_torso) / (v_2_3.len() * R_right_torso.len());
        let RShoulderRoll;
        try {
            RShoulderRoll = Math.acos(x) - Math.PI / 2;
        } catch {
            RShoulderRoll = Math.PI / 2;
        }

        return { RShoulderPitch, RShoulderRoll };
    }

    obtainLElbowYawRollAngle(P1, P5, P6, P7) {
        const v_6_7 = this.vectorFromPoints(P6, P7);
        const v_1_5 = this.vectorFromPoints(P1, P5);
        const v_6_5 = this.vectorFromPoints(P6, P5);

        const n_1_5_6 = v_1_5.cross(v_6_5);
        const R_left_arm = v_6_5.cross(n_1_5_6);

        const n_5_6_7 = v_6_5.cross(v_6_7);

        let x = n_1_5_6.dot(n_5_6_7) / (n_1_5_6.len() * n_5_6_7.len());
        let theta_LEY_module;
        try {
            theta_LEY_module = Math.acos(x);
        } catch {
            theta_LEY_module = 0;
        }

        x = v_6_7.dot(n_1_5_6) / (v_6_7.len() * n_1_5_6.len());
        let intermediateAngle1;
        try {
            intermediateAngle1 = Math.acos(x);
        } catch {
            intermediateAngle1 = Math.PI / 2;
        }

        x = v_6_7.dot(R_left_arm) / (v_6_7.len() * R_left_arm.len());
        let intermediateAngle2;
        try {
            intermediateAngle2 = Math.acos(x);
        } catch {
            intermediateAngle2 = Math.PI / 2;
        }

        let LElbowYaw = theta_LEY_module - Math.PI / 2
        // let LElbowYaw = intermediateAngle1 <= Math.PI / 2
        //     ? -theta_LEY_module
        //     : (intermediateAngle2 > Math.PI / 2 ? theta_LEY_module : theta_LEY_module - 2 * Math.PI);

        x = v_6_7.dot(v_6_5) / (v_6_7.len() * v_6_5.len());
        let LElbowRoll;
        try {
            LElbowRoll = Math.acos(x) - Math.PI;
        } catch {
            LElbowRoll = 0;
        }

        return { LElbowYaw, LElbowRoll };
    }

    obtainRElbowYawRollAngle(P1, P2, P3, P4) {
        const v_3_4 = this.vectorFromPoints(P3, P4);
        const v_1_2 = this.vectorFromPoints(P1, P2);
        const v_3_2 = this.vectorFromPoints(P3, P2);

        const n_1_2_3 = v_1_2.cross(v_3_2);
        const R_right_arm = v_3_2.cross(n_1_2_3);

        const n_2_3_4 = v_3_2.cross(v_3_4);

        let x = n_1_2_3.dot(n_2_3_4) / (n_1_2_3.len() * n_2_3_4.len());
        let theta_REY_module;
        try {
            theta_REY_module = Math.acos(x);
        } catch {
            theta_REY_module = 0;
        }

        x = v_3_4.dot(n_1_2_3) / (v_3_4.len() * n_1_2_3.len());
        let intermediateAngle1;
        try {
            intermediateAngle1 = Math.acos(x);
        } catch {
            intermediateAngle1 = Math.PI / 2;
        }

        x = v_3_4.dot(R_right_arm) / (v_3_4.len() * R_right_arm.len());
        let intermediateAngle2;
        try {
            intermediateAngle2 = Math.acos(x);
        } catch {
            intermediateAngle2 = Math.PI / 2;
        }

        let RElbowYaw = intermediateAngle1 <= Math.PI / 2
            ? -theta_REY_module
            : (intermediateAngle2 > Math.PI / 2 ? theta_REY_module : theta_REY_module);

        x = v_3_4.dot(v_3_2) / (v_3_4.len() * v_3_2.len());
        let RElbowRoll;
        try {
            RElbowRoll = Math.PI - Math.acos(x);
        } catch {
            RElbowRoll = 0;
        }

        return { RElbowYaw, RElbowRoll };
    }

    obtainHipPitchAngles(P0_curr, P8_curr) {
        const v_0_8_curr = this.vectorFromPoints(P0_curr, P8_curr);
        let n_XZ = new Vec3(0, 1, 0);
        const v_0_8_curr_proj = v_0_8_curr.sub(new Vec3(v_0_8_curr.x, 0, 0));
        let x = n_XZ.dot(v_0_8_curr_proj) / (n_XZ.len() * v_0_8_curr_proj.len());
        let omega_HP_module;
        try {
            omega_HP_module = Math.acos(x);
        } catch {
            omega_HP_module = 0;
        }

        x = v_0_8_curr_proj.dot(new Vec3(0, 0, 1)) / (v_0_8_curr_proj.len() * Math.sqrt(1));
        let intermediateAngle;
        try {
            intermediateAngle = Math.acos(x);
        } catch {
            intermediateAngle = 0;
        }

        const correction = 0.15;
        const HipPitch = intermediateAngle > Math.PI / 2
            ? Math.PI - omega_HP_module - correction
            : omega_HP_module - Math.PI - correction;

        return HipPitch;
    }

    convert(poseLandmarks, options = { unit: 'radian', visibilityThreshold: 0.5 }) {
        const angs = {};
        
        const kp_l_shoulder = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS['leftShoulder']]))
        const kp_r_shoulder = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS['rightShoulder']]))
        const kp_l_hip = poseLandmarks[BODY_JOINTS['leftHip']]
        const kp_r_hip = poseLandmarks[BODY_JOINTS['rightHip']]
        let neck = new Vec3(
            (kp_l_shoulder.x + kp_r_shoulder.x) / 2,
            (kp_l_shoulder.y + kp_r_shoulder.y) / 2,
            (kp_l_shoulder.z + kp_r_shoulder.z) / 2,
        )
        let hip = new Vec3(
            (kp_l_hip.x + kp_r_hip.x) / 2,
            (kp_l_hip.y + kp_r_hip.y) / 2,
            (kp_l_hip.z + kp_r_hip.z) / 2,
        )
        
        const kp_l_elbow = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS['leftElbow']]));
        const kp_r_elbow = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS['rightElbow']]));
        
        // debugger
        const {LShoulderPitch, LShoulderRoll} = this.obtainLShoulderPitchRollAngles(neck, kp_l_shoulder, kp_l_elbow, hip)
        angs['rightShoulder-leftShoulder-leftElbow'] = {pitch: LShoulderPitch, roll: LShoulderRoll}
        
        const kp_l_wrist = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS['leftWrist']]));
        const {LElbowYaw, LElbowRoll} = this.obtainLElbowYawRollAngle(neck, kp_l_shoulder, kp_l_elbow, kp_l_wrist)
        const LElbowPich = kp_l_wrist.sub(kp_l_elbow).rad(
            kp_l_shoulder.sub(kp_l_elbow)
        )
        angs['leftShoulder-leftElbow-leftWrist'] = {pitch: LElbowPich, yaw: LElbowYaw}
        
        const {RShoulderPitch, RShoulderRoll} = this.obtainRShoulderPitchRollAngles(neck, kp_r_shoulder, kp_r_elbow, hip)
        angs['leftShoulder-rightShoulder-rightElbow'] = {pitch: RShoulderPitch, roll: RShoulderRoll}
        
        const kp_r_wrist = new Vec3(...Object.values(poseLandmarks[BODY_JOINTS['rightWrist']]));
        const {RElbowYaw, RElbowRoll} = this.obtainRElbowYawRollAngle(neck, kp_r_shoulder, kp_r_elbow, kp_r_wrist)
        const RElbowPich = kp_r_wrist.sub(kp_r_elbow).rad(
            kp_r_shoulder.sub(kp_r_elbow)
        )
        angs['rightShoulder-rightElbow-rightWrist'] = {pitch: RElbowPich, yaw: RElbowYaw}

        
        return angs;
    }
    
    convert_hand(HandLandmarks, options = { unit: 'radian', visibilityThreshold: 0.5, keyPrefix: '' }) {
        const angs = {};
        
        const kp_thumb_tip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['thumbTIP']]))
        const kp_thumb_ip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['thumbIP']]))
        const kp_thumb_mcp = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['thumbMCP']]))
        const kp_thumb_cmc = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['thumbCMC']]))
        
        const thumb_2_pitch = kp_thumb_tip.sub(kp_thumb_ip).rad(
            kp_thumb_mcp.sub(kp_thumb_ip)
        )
        angs[options.keyPrefix + 'thumb-2'] = {pitch: thumb_2_pitch}
        
        const thumb_1_pitch = kp_thumb_ip.sub(kp_thumb_mcp).rad(
            kp_thumb_cmc.sub(kp_thumb_mcp)
        )
        angs[options.keyPrefix + 'thumb-1'] = {pitch: thumb_1_pitch}
        
        const kp_wrist = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['wrist']]))
        const kp_index_tip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['indexFingerTIP']]))
        const kp_index_dip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['indexFingerDIP']]))
        const kp_index_pip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['indexFingerPIP']]))
        const kp_index_mcp = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['indexFingerMCP']]))

        const index_0_pitch = kp_index_mcp.sub(kp_wrist).rad(
            kp_index_pip.sub(kp_index_mcp)
        )
        angs[options.keyPrefix + 'index-0'] = {pitch: index_0_pitch}
        
        const index_1_pitch = kp_index_tip.sub(kp_index_dip).rad(
            kp_index_pip.sub(kp_index_mcp)
        )
        angs[options.keyPrefix + 'index-1'] = {pitch: index_1_pitch}
        
        const kp_middle_tip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['middleFingerTIP']]))
        const kp_middle_dip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['middleFingerDIP']]))
        const kp_middle_pip = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['middleFingerPIP']]))
        const kp_middle_mcp = new Vec3(...Object.values(HandLandmarks[HAND_KEYPOINTS['middleFingerMCP']]))

        const middle_0_pitch = kp_middle_mcp.sub(kp_wrist).rad(
            kp_middle_pip.sub(kp_middle_mcp)
        )
        angs[options.keyPrefix + 'middle-0'] = {pitch: middle_0_pitch}
        
        const middle_1_pitch = kp_middle_tip.sub(kp_middle_dip).rad(
            kp_middle_pip.sub(kp_middle_mcp)
        )
        angs[options.keyPrefix + 'middle-1'] = {pitch: middle_1_pitch}
        return angs;
    }
}

// Export for usage
export const keypointsToAngles = new KeypointsToAngles();


export class PoseEst {
    
    constructor() {
        this.cretae_holistic()
        this.last_infer = 0
        this.update_interval = 100  // milliseconds
    }

    cretae_holistic () {
        const videoElement = document.getElementsByClassName('input_video')[0];
        const canvasElement = document.getElementsByClassName('output_canvas')[0];
        const canvasCtx = canvasElement.getContext('2d');

        const onResultsDraw = (results) => {
            canvasCtx.save();
            canvasCtx.clearRect(0, 0, canvasElement.width, canvasElement.height);
            if (results.segmentationMask)
                canvasCtx.drawImage(results.segmentationMask, 0, 0,
                                    canvasElement.width, canvasElement.height);

            // Only overwrite existing pixels.
            canvasCtx.globalCompositeOperation = 'source-in';
            canvasCtx.fillStyle = '#e0FF00';
            canvasCtx.fillRect(0, 0, canvasElement.width, canvasElement.height);

            // Only overwrite missing pixels.
            canvasCtx.globalCompositeOperation = 'destination-atop';
            canvasCtx.drawImage(
                results.image, 0, 0, canvasElement.width, canvasElement.height);

            canvasCtx.globalCompositeOperation = 'source-over';
            drawConnectors(canvasCtx, results.poseLandmarks, POSE_CONNECTIONS,
                          {color: '#b0FF00', lineWidth: 4});
            drawLandmarks(canvasCtx, results.poseLandmarks,
                          {color: '#FF0000', lineWidth: 2});
            drawConnectors(canvasCtx, results.faceLandmarks, FACEMESH_TESSELATION,
                          {color: '#C0C0C070', lineWidth: 1});
            drawConnectors(canvasCtx, results.leftHandLandmarks, HAND_CONNECTIONS,
                          {color: '#CC0000', lineWidth: 3});
            drawLandmarks(canvasCtx, results.leftHandLandmarks,
                          {color: '#00FF00', lineWidth: 2});
            drawConnectors(canvasCtx, results.rightHandLandmarks, HAND_CONNECTIONS,
                          {color: '#00CC00', lineWidth: 3});
            drawLandmarks(canvasCtx, results.rightHandLandmarks,
                          {color: '#FF0000', lineWidth: 2});
            canvasCtx.restore();
            
            if (results.poseLandmarks) {
                // let out = computeRelativeJointAngles(results.poseLandmarks, { unit: 'radian', visibilityThreshold: 0.5 })
                let out = keypointsToAngles.convert(results.poseLandmarks, { unit: 'radian', visibilityThreshold: 0.5 })
                // let out = medaipipeIK(results.poseLandmarks)
                if (out !== undefined && !empty_obj(out)) {
                    this.cur_joint_pos = out
                    // console.debug(this.cur_joint_pos)
                }
                if (results.leftHandLandmarks !== undefined) {
                    Object.assign(
                        this.cur_joint_pos, 
                        keypointsToAngles.convert_hand(results.leftHandLandmarks, {keyPrefix: 'left-'})
                    )
                }
                if (results.rightHandLandmarks !== undefined) {
                    Object.assign(
                        this.cur_joint_pos, 
                        keypointsToAngles.convert_hand(results.rightHandLandmarks, {keyPrefix: 'right-'})
                    )
                }
            }
        }

        this.holistic = new Holistic({locateFile: (file) => {
            return `https://cdn.jsdelivr.net/npm/@mediapipe/holistic/${file}`;
        }});
        this.holistic.setOptions({
            modelComplexity: 2,
            smoothLandmarks: true,
            enableSegmentation: true,
            smoothSegmentation: true,
            refineFaceLandmarks: true,
            minDetectionConfidence: 0.5,
            minTrackingConfidence: 0.5
        });
        this.holistic.onResults(onResultsDraw);

        const infer_loop = async () => {
            // roughly run at 10 FPS inference freqency
            if (Date.now() - this.last_infer > this.update_interval) {
                await this.holistic.send({image: videoElement});
                this.last_infer = Date.now();
            }
        }

        this.med_camera = new Camera(videoElement, {
            onFrame: infer_loop,
            width: 360,
            height: 200,
        });
        this.med_camera.start();
    }
}