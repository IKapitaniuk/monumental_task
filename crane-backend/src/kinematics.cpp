#include <kinematics.h>
#include <robot_description.h>

#include <cassert>
#include <eigen3/Eigen/Dense>

namespace assignment {

ForwardKinematicsSolver::ForwardKinematicsSolver(
    const std::vector<JointDescription>& parameters) {
  m_tcp_frame = Eigen::Isometry3d::Identity();
  m_base_frame = Eigen::Isometry3d::Identity();

  m_kinematic_chain.reserve(parameters.size());
  for (auto const& jd : parameters) {
    m_kinematic_chain.emplace_back(
        jd.name, Joint(jd.type, jd.fixed_transform, jd.joint_axis,
                       jd.limit_lower, jd.limit_upper));
  }
}

TransformMap ForwardKinematicsSolver::calc_forward_kinematics(
    const JointValues& joint_values) const {
  const auto num_joints = m_kinematic_chain.size();
  assert(num_joints == joint_values.size());
  TransformMap result;
  result.reserve(num_joints);
  Transform current_transform = m_base_frame;
  result.emplace_back("base", current_transform);

  for (size_t i = 0; i < num_joints; ++i) {
    const auto& [name, joint] = m_kinematic_chain[i];
    Transform joint_transform = joint.pose(joint_values[i]);
    current_transform = current_transform * joint_transform;
    result.emplace_back(name, current_transform);
  }
  result.emplace_back("tcp", current_transform * m_tcp_frame);
  return result;
}

Jacobian ForwardKinematicsSolver::calc_jacobian(
    const JointValues& joint_values) const {
  const auto num_joints = m_kinematic_chain.size();
  assert(num_joints == joint_values.size());

  Jacobian jac(6, num_joints);
  Transform current_transform = m_base_frame;
  std::vector<Axis> axes;
  axes.reserve(num_joints);
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(num_joints);

  for (size_t i = 0; i < num_joints; ++i) {
    const auto& [name, joint] = m_kinematic_chain[i];
    Transform joint_transform = joint.pose(joint_values[i]);
    current_transform = current_transform * joint_transform;
    Axis axis = current_transform.linear() * joint.axis();
    axes.emplace_back(axis);
    Eigen::Vector3d position = current_transform.translation();
    positions.emplace_back(position);
  }
  current_transform = current_transform * m_tcp_frame;
  Eigen::Vector3d position_tcp = current_transform.translation();

  for (size_t i = 0; i < num_joints; ++i) {
    const auto& [name, joint] = m_kinematic_chain[i];
    switch (joint.type()) {
      case JointType::Revolute: {
        jac.block(0, i, 3, 1) = axes[i].cross(position_tcp - positions[i]);
        jac.block(3, i, 3, 1) = axes[i];
      }; break;
      case JointType::Prismatic: {
        jac.block(0, i, 3, 1) = axes[i];
        jac.block(3, i, 3, 1) = Eigen::Vector3d::Zero();
      }; break;
    }
  }

  return jac;
}

}  // namespace assignment