#pragma once
#include <configuration.h>
#include <robot_description.h>
#include <robot_interface_if.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <vector>
namespace assignment {

class SimJoint {
 public:
  SimJoint(double value, double lower_limit, double upper_limit,
           double velocity_limit)
      : m_state{value},
        m_lower_limit{lower_limit},
        m_upper_limit{upper_limit},
        m_velocity_limit{velocity_limit},
        m_setpoint{value} {}

  void set_cmd(double new_value) { m_setpoint = new_value; }
  double get_state() const { return m_state; }

  double update(double dt) {
    double k_gain = 1000;
    m_state = m_state + dt * std::clamp(k_gain * (m_setpoint - m_state),
                                        -m_velocity_limit, m_velocity_limit);
    m_state = std::clamp(m_state, m_lower_limit, m_upper_limit);
    return m_state;
  }

 private:
  double m_state;
  double m_lower_limit;
  double m_upper_limit;
  double m_velocity_limit;
  double m_setpoint;
};

class SimBase {
 public:
  SimBase(double x, double y, double z, double yaw, double velocity_limit)
      : m_state{x, y, z, yaw},
        m_setpoint{x, y, z, yaw},
        m_velocity_limit{velocity_limit} {}

  void set_cmd(double x, double y, double z, double yaw) {
    m_setpoint = {x, y, z, yaw};
  }
  Transform get_state() const {
    Transform pose = Eigen::Isometry3d::Identity();
    pose.translate(Eigen::Vector3d{m_state[0], m_state[1], m_state[2]});
    pose.rotate(Eigen::AngleAxisd(m_state[3], Eigen::Vector3d::UnitZ()));

    return pose;
  }

  void update(double dt) {
    double k_gain = 1000;
    for (unsigned idx = 0; idx < 4; ++idx) {
      m_state[idx] = m_state[idx] +
                     dt * std::clamp(k_gain * (m_setpoint[idx] - m_state[idx]),
                                     -m_velocity_limit, m_velocity_limit);
    }
  }

 private:
  std::array<double, 4> m_state;
  std::array<double, 4> m_setpoint;
  double m_velocity_limit;
};

class CraneSim4DOF final : public IRobotInterface {
 public:
  CraneSim4DOF(const Configuration& config)
      : m_base{config.base.x, config.base.y, config.base.z, config.base.yaw,
               0.1} {
    m_joints.reserve(4);
    for (unsigned idx = 0; idx < 4; ++idx) {
      m_joints.emplace_back(0.0, config.joints[idx].limit_lower,
                            config.joints[idx].limit_upper,
                            config.joints[idx].max_velocity);
    }
  }

  void set_command(const JointValues& cmds) override {
    assert(cmds.size() == 4);
    for (unsigned idx = 0; idx < 4; ++idx) {
      m_joints[idx].set_cmd(cmds[idx]);
    }
  }

  void set_base_command(double x, double y, double z, double yaw) override {
    m_base.set_cmd(x, y, z, yaw);
  }

  void update(double dt) override {
    for (auto& joint : m_joints) {
      joint.update(dt);
    }
    m_base.update(dt);
  }

  JointValues get_feedback() const override {
    JointValues feedback;
    feedback.reserve(4);
    for (auto& joint : m_joints) {
      feedback.push_back(joint.get_state());
    }
    return feedback;
  }

  Transform get_base_pose() const override { return m_base.get_state(); }

 private:
  std::vector<SimJoint> m_joints;
  SimBase m_base;
};

}  // namespace assignment