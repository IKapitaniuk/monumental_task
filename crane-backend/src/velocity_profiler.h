#pragma once
#include <robot_description.h>
#include <velocity_profiler_if.h>

#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <ruckig/ruckig.hpp>
#pragma GCC diagnostic pop

namespace assignment {

class RuckigProfiler final : public IVelocityProfiler {
 public:
  RuckigProfiler(size_t dof, double dt,
                 const std::vector<JointDescription>& joints);

  void set_state(const JointValues& setpoints) override;
  void set_target(const JointValues& setpoints) override;
  JointValues update() override;

 private:
  ruckig::Ruckig<ruckig::DynamicDOFs> m_otg;
  ruckig::InputParameter<ruckig::DynamicDOFs> m_input;
  ruckig::OutputParameter<ruckig::DynamicDOFs> m_output;
};
}  // namespace assignment