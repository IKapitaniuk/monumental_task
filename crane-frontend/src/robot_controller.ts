import { JointCommand, OutcomingMessage, PoseCommand } from './message';
import { Robot } from './robot';
import * as THREE from 'three';

class TCPStateCommand {
  public x: number;
  public y: number;
  public z: number;
  public yaw: number;
  public stabilization: boolean;
  public control_algorithm: 'ik' | 'jacobian';

  constructor() {
    this.x = 0;
    this.y = 0;
    this.z = 0;
    this.yaw = 0;
    this.stabilization = false;
    this.control_algorithm = 'ik';
  }
}

interface JointStateCommand {
  value: number;
  lower_limit: number;
  upper_limit: number;
}

export class RobotController {
  private send_callback: (msg: OutcomingMessage) => void;
  public control_mode: 'joint control' | 'pose control';
  public joints: Map<string, JointStateCommand>;
  public tcp_cmd: TCPStateCommand;

  constructor(robot: Robot, cb: (msg: OutcomingMessage) => void) {
    this.send_callback = cb;
    this.control_mode = 'joint control';
    this.joints = new Map<string, JointStateCommand>();
    for (const [name, joint] of robot.joints.entries()) {
      const j: JointStateCommand = {
        value: joint.joint_value,
        lower_limit: joint.lower_limit,
        upper_limit: joint.upper_limit,
      };
      this.joints.set(name, j);
    }
    this.tcp_cmd = new TCPStateCommand();
  }

  private make_joint_msg(): JointCommand {
    const cmds = [];
    for (const [name, joint_cmd] of this.joints.entries()) {
      cmds.push({ name: name, value: joint_cmd.value });
    }

    const msg: JointCommand = {
      msg_id: 'joint_command',
      joints: cmds,
    };
    return msg;
  }

  private male_pose_msg(): PoseCommand {
    const rpy = new THREE.Euler(0, 0, (this.tcp_cmd.yaw * Math.PI) / 180);
    const q = new THREE.Quaternion().setFromEuler(rpy);

    const msg: PoseCommand = {
      msg_id: 'pose_command',
      tcp_pose: {
        position: {
          x: this.tcp_cmd.x,
          y: this.tcp_cmd.y,
          z: this.tcp_cmd.z,
        },
        quaternion: {
          x: q.x,
          y: q.y,
          z: q.z,
          w: q.w,
        },
      },
      stabilization: this.tcp_cmd.stabilization,
      control_algorithm: this.tcp_cmd.control_algorithm,
    };
    return msg;
  }

  public send_command() {
    if (this.control_mode === 'joint control') {
      const msg = this.make_joint_msg();
      this.send_callback(msg);
    } else if (this.control_mode === 'pose control') {
      const msg = this.male_pose_msg();
      this.send_callback(msg);
    }
  }
}
