#pragma once

#include <robot_description.h>

#include <memory>

namespace assignment {

class IVelocityProfiler {
 public:
  using UPtr = std::unique_ptr<IVelocityProfiler>;

  virtual ~IVelocityProfiler() {}

  virtual void set_state(const JointValues& setpoints) = 0;
  virtual void set_target(const JointValues& setpoints) = 0;
  virtual JointValues update() = 0;
};

}  // namespace assignment