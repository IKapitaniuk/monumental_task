#include <velocity_profiler.h>

#include <cassert>

namespace assignment {

RuckigProfiler::RuckigProfiler(size_t dof, double dt,
                               const std::vector<JointDescription>& joints)
    : m_otg{dof, dt}, m_input{dof}, m_output{dof} {
  for (size_t idx = 0; idx < dof; ++idx) {
    m_input.current_position[idx] = 0;
    m_input.current_velocity[idx] = 0;
    m_input.current_acceleration[idx] = 0;
    m_input.target_position[idx] = 0;
    m_input.target_velocity[idx] = 0;
    m_input.target_acceleration[idx] = 0;

    m_input.max_velocity[idx] = joints[idx].max_velocity;
    m_input.max_acceleration[idx] = joints[idx].max_acceleration;
    // m_input.max_jerk[idx] = 30;
  }
}

void RuckigProfiler::set_state(const JointValues& setpoints) {
  assert(setpoints.size() == 4);
  for (unsigned idx = 0; idx < setpoints.size(); ++idx) {
    m_input.current_position[idx] = setpoints[idx];
  }
}

void RuckigProfiler::set_target(const JointValues& setpoints) {
  assert(setpoints.size() == 4);
  for (unsigned idx = 0; idx < setpoints.size(); ++idx) {
    m_input.target_position[idx] = setpoints[idx];
  }
}

JointValues RuckigProfiler::update() {
  JointValues command(4);
  auto state = m_otg.update(m_input, m_output);
  for (unsigned idx = 0; idx < command.size(); ++idx) {
    command[idx] = m_output.new_position[idx];
  }
  if (state == ruckig::Result::Working) {
    m_output.pass_to_input(m_input);
  }
  return command;
}

}  // namespace assignment