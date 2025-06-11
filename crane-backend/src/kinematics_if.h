#pragma once
#include <robot_description.h>

#include <memory>

namespace assignment {

class IForwardKinematicsSolver {
 public:
  using UPtr = std::unique_ptr<IForwardKinematicsSolver>;

  virtual ~IForwardKinematicsSolver() {}

  virtual void set_tcp_frame(const Transform& new_tcp_frame) = 0;
  virtual void set_base_frame(const Transform& new_tcp_frame) = 0;
  virtual TransformMap calc_forward_kinematics(
      const JointValues& joint_values) const = 0;
  virtual Jacobian calc_jacobian(const JointValues& joint_values) const = 0;
};

class IInverseKinematicsSolver {
 public:
  using UPtr = std::unique_ptr<IInverseKinematicsSolver>;

  virtual ~IInverseKinematicsSolver() {}

  virtual std::vector<JointValues> calc_inverse_kinematics(
      const Transform&) const = 0;
};

}  // namespace assignment