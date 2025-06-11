#pragma once
#include <robot_description.h>

namespace assignment {

class IRobotInterface {
 public:
  virtual ~IRobotInterface() {}

  virtual void set_command(const JointValues&) = 0;
  virtual JointValues get_feedback() const = 0;
  virtual void update(double dt) = 0;
  virtual void set_base_command(double x, double y, double z, double yaw) = 0;
  virtual Transform get_base_pose() const = 0;
};

}  // namespace assignment