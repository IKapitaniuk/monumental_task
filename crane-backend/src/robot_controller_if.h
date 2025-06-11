#pragma once
#include <robot_description.h>

namespace assignment {

enum class ControlAlgorithm {
  ik,
  jacobian,
};

class IRobotController {
 public:
  virtual ~IRobotController(){};
  virtual void set_tcp_frame(const Transform&) = 0;
  virtual void set_base_frame(const Transform&) = 0;

  virtual void set_joint_setpoint(const JointValues&) = 0;
  virtual void set_pose_setpoint(const Transform&) = 0;

  virtual void set_feedback_state(const JointValues&) = 0;
  virtual void update() = 0;

  virtual void enable_stabilization(bool on) = 0;
  virtual bool enabled_stabilization() const = 0;

  virtual JointValues get_joint_setpoint() const = 0;
  virtual Transform get_tcp_pose() const = 0;

  virtual void set_control_algorithm(ControlAlgorithm) = 0;
};

}  // namespace assignment