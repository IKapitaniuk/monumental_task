import { GUI } from 'three/examples/jsm/libs/lil-gui.module.min.js';
import { Robot } from './robot';
import { RobotController } from './robot_controller';
import * as THREE from 'three';

export class RobotGUI {
  private robot_folder: GUI;

  constructor(
    robot: Robot,
    controller: RobotController,
    target: THREE.Mesh,
    gui: GUI
  ) {
    this.robot_folder = gui.addFolder(robot.name);
    new StateGUI(robot, this.robot_folder);
    new ControlGUI(controller, target, this.robot_folder);
  }

  public remove(): void {
    this.robot_folder.destroy();
  }
}

class StateGUI {
  constructor(robot: Robot, gui: GUI) {
    const joint_folder = gui.addFolder('Joints State');
    for (const [name, joint] of robot.joints.entries()) {
      joint_folder
        .add(joint, 'joint_value', joint.lower_limit, joint.upper_limit, 0.01)
        .name(name)
        .listen()
        .disable();
    }

    const tcp_folder = gui.addFolder('TCP State');
    tcp_folder.add(robot.tcp_state, 'x').name('tcp x').listen().disable();
    tcp_folder.add(robot.tcp_state, 'y').name('tcp y').listen().disable();
    tcp_folder.add(robot.tcp_state, 'z').name('tcp z').listen().disable();
    tcp_folder.add(robot.tcp_state, 'yaw').name('tcp yaw').listen().disable();
  }
}

class ControlGUI {
  private control_tab: GUI;
  private controller: RobotController;
  private target: THREE.Mesh;

  constructor(controller: RobotController, target: THREE.Mesh, gui: GUI) {
    this.controller = controller;
    const control_gui = gui.addFolder('Control Mode');
    this.control_tab = gui.addFolder('Robot Control');
    this.target = target;

    control_gui
      .add(this.controller, 'control_mode', ['joint control', 'pose control'])
      .onChange((mode: 'joint control' | 'pose control') => {
        this.control_tab.destroy();
        this.control_tab = gui.addFolder('Robot Control');
        if (mode === 'joint control') {
          this.set_joint_control_gui();
        } else if (mode === 'pose control') {
          this.set_pose_control_gui();
        }
        this.control_tab.add(controller, 'send_command').name('Send Command');
      })
      .reset();
  }

  private set_joint_control_gui() {
    this.target.visible = false;
    for (const [name, joint] of this.controller.joints.entries()) {
      this.control_tab
        .add(joint, 'value', joint.lower_limit, joint.upper_limit, 0.01)
        .name(name);
    }
  }

  private set_pose_control_gui() {
    this.target.visible = true;
    this.control_tab.add(this.controller.tcp_cmd, 'stabilization');
    this.control_tab
      .add(this.controller.tcp_cmd, 'control_algorithm', ['ik', 'jacobian'])
      .name('stabilization algorithm');

    this.control_tab
      .add(this.controller.tcp_cmd, 'x', -2, 2, 0.01)
      .name('tcp x')
      .onChange((value) => {
        this.target.position.x = value;
      })
      .reset();

    this.control_tab
      .add(this.controller.tcp_cmd, 'y', -2, 2, 0.01)
      .name('tcp y')
      .onChange((value) => {
        this.target.position.y = value;
      })
      .reset();

    this.control_tab
      .add(this.controller.tcp_cmd, 'z', -2, 2, 0.01)
      .name('tcp z')
      .onChange((value) => {
        this.target.position.z = value;
      })
      .reset();

    this.control_tab
      .add(this.controller.tcp_cmd, 'yaw', -180, 180, 0.01)
      .name('tcp yaw')
      .onChange((value) => {
        this.target.rotation.z = (value * Math.PI) / 180;
      })
      .reset();
  }
}
