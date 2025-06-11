#include <robot_controller.h>

namespace assignment {

RobotController4DOF::RobotController4DOF(
    double dt, IForwardKinematicsSolver::UPtr&& fk_solver,
    IInverseKinematicsSolver::UPtr&& ik_solver,
    IVelocityProfiler::UPtr&& profiler)
    : m_dt{dt},
      m_stabilization_on{false},
      m_fk_solver{std::move(fk_solver)},
      m_ik_solver{std::move(ik_solver)},
      m_profiler{std::move(profiler)} {
  m_tcp_frame = Eigen::Isometry3d::Identity();
  m_base_frame = Eigen::Isometry3d::Identity();
  m_tcp_goal = Eigen::Isometry3d::Identity();
  m_fk_solver->set_tcp_frame(m_tcp_frame);
  m_fk_solver->set_base_frame(m_base_frame);
  m_joints_state.resize(4);  // 4DoF arm
  m_control_algorithm = [this]() { this->control_ik(); };
}

void RobotController4DOF::set_control_algorithm(ControlAlgorithm algorithm) {
  switch (algorithm) {
    case ControlAlgorithm::ik: {
      m_control_algorithm = [this]() { this->control_ik(); };
    } break;

    case ControlAlgorithm::jacobian: {
      m_control_algorithm = [this]() { this->control_jac(); };
    } break;
  }
}

void RobotController4DOF::set_tcp_frame(const Transform& new_tcp_frame) {
  m_tcp_frame = new_tcp_frame;
  m_fk_solver->set_tcp_frame(m_tcp_frame);
}

void RobotController4DOF::set_base_frame(const Transform& new_base_frame) {
  m_base_frame = new_base_frame;
  m_fk_solver->set_base_frame(m_base_frame);
}

JointValues RobotController4DOF::choose_solution(
    const std::vector<JointValues>& solutions, const JointValues& state) const {
  if (solutions.size() == 1) {
    return solutions[0];
  }
  auto functional = [](const JointValues& a, const JointValues& b) {
    assert(a.size() == b.size());
    double J = 0.0;
    for (unsigned idx = 0; idx < a.size(); ++idx) {
      J += std::abs(a[idx] - b[idx]);
    }
    return J;
  };
  auto fun_1 = functional(state, solutions[0]);
  auto fun_2 = functional(state, solutions[1]);
  return fun_1 < fun_2 ? solutions[0] : solutions[1];
}

void RobotController4DOF::control_ik() {
  Transform arm_pose =
      m_base_frame.inverse() * m_tcp_goal * m_tcp_frame.inverse();
  auto ik_solutions = m_ik_solver->calc_inverse_kinematics(arm_pose);
  if (ik_solutions.size() == 0) {
    return;  // no solutions
  }
  const auto joints_setspoint = choose_solution(ik_solutions, m_joints_state);
  m_profiler->set_target(joints_setspoint);
}

void RobotController4DOF::control_jac() {
  auto fk = m_fk_solver->calc_forward_kinematics(m_joints_state);
  auto tcp_pose = std::get<1>(fk.back());
  auto jac = m_fk_solver->calc_jacobian(m_joints_state);

  Eigen::Matrix4d jac4;
  jac4.block(0, 0, 3, 4) = jac.block(0, 0, 3, 4);
  jac4.block(3, 0, 1, 4) = jac.block(5, 0, 1, 4);

  Eigen::Vector3d des_pos = m_tcp_goal.translation();
  Eigen::Vector3d des_rpy = m_tcp_goal.linear().eulerAngles(0, 1, 2);

  Eigen::Vector3d cur_pos = tcp_pose.translation();
  Eigen::Vector3d cur_rpy = tcp_pose.linear().eulerAngles(0, 1, 2);

  double k_gain = 1000;
  Eigen::Vector4d u{
      k_gain * (des_pos[0] - cur_pos[0]),
      k_gain * (des_pos[1] - cur_pos[1]),
      k_gain * (des_pos[2] - cur_pos[2]),
      k_gain * (des_rpy[2] - cur_rpy[2]),
  };

  Eigen::Vector4d dq;
  double det = jac4.determinant();
  if (std::abs(det) < 1e-10) {
    dq = Eigen::Matrix4d::Identity() * u;
  } else {
    dq = jac4.inverse() * u;
  }

  JointValues new_q = {
      m_joints_state[0] + m_dt * dq[0],
      m_joints_state[1] + m_dt * dq[1],
      m_joints_state[2] + m_dt * dq[2],
      m_joints_state[3] + m_dt * dq[3],
  };

  m_profiler->set_target(new_q);
}

void RobotController4DOF::set_feedback_state(const JointValues& joints) {
  assert(joints.size() == m_joints_state.size());
  assert(joints.size() == 4);

  for (unsigned idx = 0; idx < m_joints_state.size(); ++idx) {
    m_joints_state[idx] = joints[idx];
  }
}

void RobotController4DOF::set_joint_setpoint(
    const JointValues& desired_joint_values) {
  assert(desired_joint_values.size() == m_joints_goal.size());
  assert(desired_joint_values.size() == 4);

  m_profiler->set_state(m_joints_state);
  m_profiler->set_target(desired_joint_values);
}

void RobotController4DOF::set_pose_setpoint(const Transform& tcp_pose) {
  m_profiler->set_state(m_joints_state);
  m_tcp_goal = tcp_pose;
  control_ik();
}

void RobotController4DOF::update() {
  if (m_stabilization_on) {
    m_control_algorithm();
  }
  m_joints_state = m_profiler->update();
}

JointValues RobotController4DOF::get_joint_setpoint() const {
  return m_joints_state;
}

Transform RobotController4DOF::get_tcp_pose() const {
  TransformMap fk = m_fk_solver->calc_forward_kinematics(m_joints_state);
  auto [name, tf] = fk.back();
  assert(name == "tcp");
  return tf;
}

void RobotController4DOF::enable_stabilization(bool on) {
  m_stabilization_on = on;
  m_profiler->set_state(m_joints_state);
}

bool RobotController4DOF::enabled_stabilization() const {
  return m_stabilization_on;
}

}  // namespace assignment
