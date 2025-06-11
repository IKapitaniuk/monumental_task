export interface Pose {
  position: { x: number; y: number; z: number };
  quaternion: { x: number; y: number; z: number; w: number };
}

export interface StateMessage {
  msg_id: 'state';
  joints: { name: string; value: number }[];
  tcp_pose: Pose;
  base_pose: Pose;
}

export interface ConfigurationMessage {
  msg_id: 'configuration_update';
  configuration: string;
}

export type IncomingMessage = StateMessage | ConfigurationMessage;

export interface JointCommand {
  msg_id: 'joint_command';
  joints: { name: string; value: number }[];
}

export interface PoseCommand {
  msg_id: 'pose_command';
  tcp_pose: Pose;
  stabilization: boolean;
  control_algorithm: 'ik' | 'jacobian';
}

export interface ConfigurationRequest {
  msg_id: 'configuration_request';
}

// export interface PoseCommand {
//   msg_id: 'pose_command';
//   tcp_pose: Pose;
// }

export type OutcomingMessage =
  | JointCommand
  | PoseCommand
  | ConfigurationRequest;
