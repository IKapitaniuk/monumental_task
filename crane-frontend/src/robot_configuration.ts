type Vec3 = [number, number, number];

export interface JointConfiguration {
  name: string;
  type: 'revolute' | 'prismatic';
  axis: Vec3;
  position: Vec3;
  rotation_rpy: Vec3;
  lower_limit?: number;
  upper_limit?: number;
  max_velocity: number;
  max_acceleration: number;
}

export interface BoxGeometry {
  id: 'box';
  x: number;
  y: number;
  z: number;
}

export interface CylinderGeometry {
  id: 'cylinder';
  radius: number;
  height: number;
}

export interface LinkConfiguration {
  name: string;
  joint: string;
  geometry: BoxGeometry | CylinderGeometry;
  position: Vec3;
  //   rotation_rpy: Vec3;
}

export interface RobotConfiguration {
  name: string;
  joints: JointConfiguration[];
  links: LinkConfiguration[];
  tcp: TCPConfiguration;
  base: BaseConfiguration;
  ik: IKSolverConfiguration;
}

export interface TCPConfiguration {
  name: 'tcp';
  position: Vec3;
  rotation_rpy: Vec3;
}

export interface BaseConfiguration {
  name: 'base';
  geometry: BoxGeometry | CylinderGeometry;
  position: Vec3;
  yaw: number;
}

export interface IKSolverConfiguration {
  lift_offset: Vec3;
  elbow_offset: Vec3;
  wrist_offset: Vec3;
}

export function test_configuration(): RobotConfiguration {
  const joint0: JointConfiguration = {
    name: 'swing',
    type: 'revolute',
    axis: [0, 0, 1],
    position: [0, 0, 0],
    rotation_rpy: [0, 0, 0],
    max_velocity: 0.2,
    max_acceleration: 3.0,
  };

  const joint1: JointConfiguration = {
    name: 'lift',
    type: 'prismatic',
    axis: [0, 0, 1],
    position: [0, 0, 0.2],
    rotation_rpy: [0, 0, 0],
    lower_limit: 0.0,
    upper_limit: 1.65,
    max_velocity: 0.2,
    max_acceleration: 3.0,
  };

  const joint2: JointConfiguration = {
    name: 'elbow',
    type: 'revolute',
    axis: [0, 0, 1],
    position: [0.5, 0, -0.1],
    rotation_rpy: [0, 0, 0],
    lower_limit: -2.5,
    upper_limit: +2.5,
    max_velocity: 0.2,
    max_acceleration: 3.0,
  };

  const joint3: JointConfiguration = {
    name: 'wrist',
    type: 'revolute',
    axis: [0, 0, 1],
    position: [0.5, 0, -0.2],
    rotation_rpy: [0, 0, 0],
    max_velocity: 0.2,
    max_acceleration: 3.0,
  };

  const tcp: TCPConfiguration = {
    name: 'tcp',
    position: [0.2, 0, 0],
    rotation_rpy: [0, 0, 0],
  };

  const link0: LinkConfiguration = {
    name: 'lift column',
    joint: 'swing',
    position: [0, 0, 1.0],
    geometry: { id: 'box', x: 0.1, y: 0.1, z: 2.0 },
  };

  const link1: LinkConfiguration = {
    name: 'lift link1',
    joint: 'lift',
    position: [0, 0, 0.0],
    geometry: { id: 'box', x: 0.2, y: 0.2, z: 0.2 },
  };

  const link2: LinkConfiguration = {
    name: 'lift link2',
    joint: 'lift',
    position: [0.3, 0, 0.0],
    geometry: { id: 'box', x: 0.6, y: 0.1, z: 0.1 },
  };

  const link3: LinkConfiguration = {
    name: 'elbow link',
    joint: 'elbow',
    position: [0.25, 0, 0.0],
    geometry: { id: 'box', x: 0.6, y: 0.1, z: 0.09 },
  };

  const link4: LinkConfiguration = {
    name: 'wrist link',
    joint: 'wrist',
    position: [0, 0, 0.07],
    geometry: { id: 'box', x: 0.1, y: 0.1, z: 0.15 },
  };

  const link5: LinkConfiguration = {
    name: 'gripper',
    joint: 'wrist',
    position: [0.1, 0, 0.0],
    geometry: { id: 'box', x: 0.2, y: 0.1, z: 0.01 },
  };

  const base: BaseConfiguration = {
    name: 'base',
    position: [0, 0, 0.2],
    yaw: 0,
    geometry: { id: 'box', x: 1, y: 0.5, z: 0.2 },
  };

  const ik: IKSolverConfiguration = {
    lift_offset: [0, 0, 0.2],
    elbow_offset: [0.5, 0.0, -0.1],
    wrist_offset: [0.5, 0.0, -0.2],
  };

  const config: RobotConfiguration = {
    name: 'test_robot',
    joints: [joint0, joint1, joint2, joint3],
    links: [link0, link1, link2, link3, link4, link5],
    tcp,
    base,
    ik,
  };
  return config;
}
