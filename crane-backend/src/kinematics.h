#pragma once

#include <kinematics_if.h>
#include <robot_description.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

namespace assignment {

class Joint {
 public:
  Joint(JointType type, const Transform& fixed_transform, const Axis& axis,
        double low_limit, double high_limit)
      : m_type{type},
        m_fixed_transform{fixed_transform},
        m_axis{axis},
        m_low_limit{low_limit},
        m_high_limit{high_limit} {}

  Transform pose(double joint_value) const {
    double clamped_value = std::clamp(joint_value, m_low_limit, m_high_limit);
    Transform result = m_fixed_transform;
    if (m_type == JointType::Prismatic) {
      result = result * Eigen::Translation3d(clamped_value * m_axis);
    } else if (m_type == JointType::Revolute) {
      result = result * Eigen::AngleAxisd(clamped_value, m_axis);
    }
    return result;
  }

  Axis axis() const { return m_axis; }
  JointType type() const { return m_type; }

 private:
  JointType m_type;
  Transform m_fixed_transform;
  Axis m_axis;
  double m_low_limit;
  double m_high_limit;
};

using Chain = std::vector<std::tuple<std::string, Joint>>;

class ForwardKinematicsSolver final : public IForwardKinematicsSolver {
 public:
  ForwardKinematicsSolver(const std::vector<JointDescription>& parameters);

  void set_tcp_frame(const Transform& new_tcp_frame) override {
    m_tcp_frame = new_tcp_frame;
  }

  void set_base_frame(const Transform& new_base_frame) override {
    m_base_frame = new_base_frame;
  }

  TransformMap calc_forward_kinematics(
      const JointValues& joint_values) const override;
  Jacobian calc_jacobian(const JointValues& joint_values) const override;

 private:
  Chain m_kinematic_chain;
  Transform m_tcp_frame;
  Transform m_base_frame;
};

class InverseKinematicsSolver4DOF final : public IInverseKinematicsSolver {
 public:
  InverseKinematicsSolver4DOF(const IKSolverDescription& parameters,
                              const std::vector<JointDescription>& joints) {
    assert(joints.size() == 4);
    auto& p = parameters;
    m_z = p.lift_offset[2] + p.elbow_offset[2] + p.wrist_offset[2];
    double x1 = p.lift_offset[0] + p.elbow_offset[0];
    double y1 = p.lift_offset[1] + p.elbow_offset[1];
    double x2 = p.wrist_offset[0];
    double y2 = p.wrist_offset[1];
    m_phi_1 = atan2(y1, x1);
    m_l_1 = hypot(x1, y1);
    m_phi_2 = atan2(y2, x2);
    m_l_2 = hypot(x2, y2);
    for (unsigned idx = 0; idx < 4; ++idx) {
      m_lower_limits[idx] = joints[idx].limit_lower;
      m_upper_limits[idx] = joints[idx].limit_upper;
    };
  }

  std::vector<JointValues> calc_inverse_kinematics(
      const Transform& desired_pose) const override {
    //
    std::vector<JointValues> result;
    result.reserve(2);

    Eigen::Vector3d desired_position = desired_pose.translation();
    Eigen::Vector3d desired_orientation =
        desired_pose.rotation().eulerAngles(0, 1, 2);

    const double z = desired_position[2];
    const double q_2 = z - m_z;  // Lift Joint Value

    const double x = desired_position[0];
    const double y = desired_position[1];
    const double yaw = desired_orientation[2];

    const double cosine =
        (x * x + y * y - m_l_1 * m_l_1 - m_l_2 * m_l_2) / (2 * m_l_1 * m_l_2);
    if (std::abs(cosine) > 1.0) {
      return result;  // no solutions
    }
    const double sine = sqrt(1 - cosine * cosine);
    const double q_3_1 = atan2(sine, cosine) + m_phi_1 - m_phi_2;   // 1st case
    const double q_3_2 = atan2(-sine, cosine) + m_phi_1 - m_phi_2;  // 2nd case

    const double k_1 = m_l_1 + m_l_2 * cosine;
    const double k_2 = m_l_2 * sine;

    const double a = k_1 * y - k_2 * x;
    const double b = k_1 * x + k_2 * y;
    const double c = k_1 * y + k_2 * x;
    const double d = k_1 * x - k_2 * y;

    const double q_1_1 = atan2(a, b) - m_phi_1;  // 1st case
    const double q_1_2 = atan2(c, d) - m_phi_1;  // 2nd case

    const double q_4_1 = yaw - q_1_1 - q_3_1;  // 1st case
    const double q_4_2 = yaw - q_1_2 - q_3_2;  // 1st case

    std::vector<double> solution1 = {q_1_1, q_2, q_3_1, q_4_1};
    if (check_limit(solution1)) {
      result.push_back(std::move(solution1));
    }

    std::vector<double> solution2 = {q_1_2, q_2, q_3_2, q_4_2};
    if (check_limit(solution2)) {
      result.push_back(std::move(solution2));
    }

    return result;
  }

 private:
  bool check_limit(const std::vector<double>& solution) const {
    bool within_limits = true;
    for (unsigned idx = 0; idx < 4; ++idx) {
      within_limits &= (m_lower_limits[idx] <= solution[idx] &&
                        solution[idx] <= m_upper_limits[idx]);
    }
    return within_limits;
  }

  double m_phi_1;
  double m_l_1;
  double m_phi_2;
  double m_l_2;
  double m_z;
  std::array<double, 4> m_lower_limits;
  std::array<double, 4> m_upper_limits;
};

}  // namespace assignment