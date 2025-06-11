import {
  BaseConfiguration,
  JointConfiguration,
  LinkConfiguration,
  RobotConfiguration,
  TCPConfiguration,
} from './robot_configuration';
import * as THREE from 'three';

import { StateMessage } from './message';

type JoinType = 'revolute' | 'prismatic';

class Joint extends THREE.Object3D {
  private axis: THREE.Vector3;
  private fixed_transformation: THREE.Matrix4;
  private joint_type: JoinType;
  public readonly lower_limit: number;
  public readonly upper_limit: number;
  public joint_value: number;

  constructor(config: JointConfiguration, value: number = 0.0) {
    super();
    this.name = config.name;

    this.axis = new THREE.Vector3(
      config.axis[0],
      config.axis[1],
      config.axis[2]
    );
    const position = new THREE.Vector3(
      config.position[0],
      config.position[1],
      config.position[2]
    );
    const orientation = new THREE.Quaternion();
    const rpy = new THREE.Euler(
      config.rotation_rpy[0],
      config.rotation_rpy[1],
      config.rotation_rpy[2]
    );
    orientation.setFromEuler(rpy);
    this.fixed_transformation = new THREE.Matrix4().compose(
      position,
      orientation,
      new THREE.Vector3(1, 1, 1)
    );
    this.joint_type = config.type;
    this.joint_value = value;
    this.add(new THREE.AxesHelper(0.3));

    if (config.lower_limit !== undefined) {
      this.lower_limit = config.lower_limit;
    } else {
      this.lower_limit = -Math.PI * 2;
    }

    if (config.upper_limit !== undefined) {
      this.upper_limit = config.upper_limit;
    } else {
      this.upper_limit = +Math.PI * 2;
    }

    this.update_joint_value(value);
  }

  public update_joint_value(joint_value: number) {
    this.joint_value = joint_value;
    if (this.joint_type === 'revolute') {
      const new_matrix = new THREE.Matrix4()
        .makeRotationAxis(this.axis, this.joint_value)
        .premultiply(this.fixed_transformation);
      new_matrix.decompose(this.position, this.quaternion, this.scale);
    } else if (this.joint_type == 'prismatic') {
      const new_translation = this.axis
        .clone()
        .multiplyScalar(this.joint_value);
      const new_matrix = new THREE.Matrix4()
        .makeTranslation(new_translation)
        .premultiply(this.fixed_transformation);
      new_matrix.decompose(this.position, this.quaternion, this.scale);
    }
  }
}

class TCP extends THREE.Object3D {
  private fixed_transformation: THREE.Matrix4;

  constructor(config: TCPConfiguration) {
    super();
    this.name = config.name;

    const position = new THREE.Vector3(
      config.position[0],
      config.position[1],
      config.position[2]
    );
    const orientation = new THREE.Quaternion();
    const rpy = new THREE.Euler(
      config.rotation_rpy[0],
      config.rotation_rpy[1],
      config.rotation_rpy[2]
    );
    orientation.setFromEuler(rpy);
    this.fixed_transformation = new THREE.Matrix4().compose(
      position,
      orientation,
      new THREE.Vector3(1, 1, 1)
    );

    this.fixed_transformation.decompose(
      this.position,
      this.quaternion,
      this.scale
    );
    this.add(new THREE.AxesHelper(0.5));
  }
}

class Link extends THREE.Object3D {
  // private mesh: THREE.Mesh;

  constructor(config: LinkConfiguration) {
    super();
    this.name = config.name;
    const geometry = config.geometry;
    const material = new THREE.MeshNormalMaterial();
    let mesh: THREE.Mesh | null = null;
    if (geometry.id === 'box') {
      const box = new THREE.BoxGeometry(geometry.x, geometry.y, geometry.z);
      mesh = new THREE.Mesh(box, material);
    } else if (geometry.id === 'cylinder') {
      const cylinder = new THREE.CylinderGeometry(
        geometry.radius,
        geometry.radius,
        geometry.height
      );
      mesh = new THREE.Mesh(cylinder, material);
    }
    if (mesh !== null) {
      mesh.position.x = config.position[0];
      mesh.position.y = config.position[1];
      mesh.position.z = config.position[2];

      this.add(mesh);
    }
  }
}

class Base extends THREE.Object3D {
  constructor(config: BaseConfiguration) {
    super();
    this.name = config.name;
    this.position.x = config.position[0];
    this.position.y = config.position[1];
    this.position.z = config.position[2];
    this.rotation.z = config.yaw;

    const geometry = config.geometry;
    const material = new THREE.MeshNormalMaterial();
    let mesh: THREE.Mesh | null = null;
    if (geometry.id === 'box') {
      const box = new THREE.BoxGeometry(geometry.x, geometry.y, geometry.z);
      mesh = new THREE.Mesh(box, material);
      mesh.position.z = -geometry.z / 2;
    } else if (geometry.id === 'cylinder') {
      const cylinder = new THREE.CylinderGeometry(
        geometry.radius,
        geometry.radius,
        geometry.height
      );
      mesh = new THREE.Mesh(cylinder, material);
      mesh.position.z = -geometry.height / 2;
    }
    if (mesh !== null) {
      this.add(mesh);
    }
  }
}

class TCPState {
  public x: number;
  public y: number;
  public z: number;
  public yaw: number;
  private tcp: TCP;

  constructor(tcp: TCP) {
    this.tcp = tcp;
    this.x = 0;
    this.y = 0;
    this.z = 0;
    this.yaw = 0;
    this.update();
  }

  public update() {
    const position = new THREE.Vector3();
    this.tcp.getWorldPosition(position);
    this.x = Math.round(position.x * 1000) / 1000;
    this.y = Math.round(position.y * 1000) / 1000;
    this.z = Math.round(position.z * 1000) / 1000;

    const orientation = new THREE.Quaternion();
    this.tcp.getWorldQuaternion(orientation);
    const euler = new THREE.Euler();
    euler.setFromQuaternion(orientation);
    this.yaw = Math.round(((euler.z * 180.0) / Math.PI) * 100) / 100;
  }
}

export class Robot {
  public readonly name: string;
  public readonly tcp: TCP;
  public readonly joints: Map<string, Joint>;
  private base: Base;
  public tcp_state: TCPState;

  constructor(name: string, config: RobotConfiguration) {
    this.name = name;
    // Base
    this.base = new Base(config.base);
    // Joints
    this.joints = new Map<string, Joint>();
    let prev_joint = this.base;
    for (const jnt_cfg of config.joints) {
      const joint = new Joint(jnt_cfg, 0.0);
      prev_joint.add(joint);
      prev_joint = joint;
      this.joints.set(jnt_cfg.name, joint);
    }
    // TCP
    this.tcp = new TCP(config.tcp);
    prev_joint.add(this.tcp);
    this.tcp_state = new TCPState(this.tcp);
    // Links
    for (const link_cfg of config.links) {
      const link = new Link(link_cfg);
      const joint_to_attach = link_cfg.joint;
      const joint = this.joints.get(joint_to_attach);
      joint?.add(link);
    }
  }

  public set_state(state: StateMessage) {
    for (const joint of state.joints) {
      const val = Math.round(joint.value * 1000) / 1000;
      const j = this.joints.get(joint.name);
      j?.update_joint_value(val);
    }

    this.base.position.x = state.base_pose.position.x;
    this.base.position.y = state.base_pose.position.y;
    this.base.position.z = state.base_pose.position.z;
    this.base.quaternion.x = state.base_pose.quaternion.x;
    this.base.quaternion.y = state.base_pose.quaternion.y;
    this.base.quaternion.z = state.base_pose.quaternion.z;
    this.base.quaternion.w = state.base_pose.quaternion.w;
    this.base.quaternion.normalize();

    this.tcp_state.update();
  }

  public get_root(): THREE.Object3D {
    return this.base;
  }
}
