import './style.css';

import { Application } from './application';
import { Scene3D } from './scene';
// import { test_configuration } from './robot_configuration';
// import { Robot } from './robot';
import GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import * as THREE from 'three';
import { Pose } from './message';

const gui = new GUI();
const scene = new Scene3D();
const application = new Application(gui, scene);

const debug = document.getElementById('debug') as HTMLDivElement;

function animate() {
  requestAnimationFrame(animate);

  scene.animate();

  const state = application.get_state();

  let state_str = 'Backend Data\n';
  if (state !== null) {
    state_str += '\nJoints:\n';
    for (const joint of state.joints) {
      state_str += `${joint.name}: ${Number(joint.value).toFixed(3)}\n`;
    }
    state_str += '\nTCP Pose:\n';
    state_str = print_pose(state_str, state.tcp_pose);

    state_str += '\nBase Pose:\n';
    state_str = print_pose(state_str, state.base_pose);
  }

  debug.innerText = state_str;
}

function print_pose(str: string, pose: Pose): string {
  str += `x: ${Number(pose.position.x).toFixed(3)}\n`;
  str += `y: ${Number(pose.position.y).toFixed(3)}\n`;
  str += `z: ${Number(pose.position.z).toFixed(3)}\n`;

  const q = new THREE.Quaternion(
    pose.quaternion.x,
    pose.quaternion.y,
    pose.quaternion.z,
    pose.quaternion.w
  );
  q.normalize();
  const rpy = new THREE.Euler().setFromQuaternion(q);
  str += `yaw: ${Number((rpy.z * 180) / Math.PI).toFixed(3)}\n`;
  return str;
}

animate();
