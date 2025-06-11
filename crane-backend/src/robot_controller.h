#pragma once
#include <kinematics_if.h>
#include <robot_controller_if.h>
#include <robot_description.h>
#include <velocity_profiler_if.h>

#include <cassert>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <memory>

namespace assignment {

class RobotController4DOF final : public IRobotController {
 public:
  RobotController4DOF(double dt, IForwardKinematicsSolver::UPtr&& fk_solver,
                      IInverseKinematicsSolver::UPtr&& ik_solver,
                      IVelocityProfiler::UPtr&& profiler);

  void set_tcp_frame(const Transform& new_tcp_frame) override;
  void set_base_frame(const Transform& new_base_frame) override;
  void set_feedback_state(const JointValues& joints) override;
  void set_joint_setpoint(const JointValues& desired_joint_values) override;
  void set_pose_setpoint(const Transform& tcp_pose) override;
  void update() override;
  JointValues get_joint_setpoint() const override;
  Transform get_tcp_pose() const override;

  void enable_stabilization(bool on) override;
  bool enabled_stabilization() const override;

  void set_control_algorithm(ControlAlgorithm) override;

 private:
  void control_jac();
  void control_ik();
  JointValues choose_solution(const std::vector<JointValues>& solutions,
                              const JointValues& state) const;

  double m_dt;
  bool m_stabilization_on;
  IForwardKinematicsSolver::UPtr m_fk_solver;
  IInverseKinematicsSolver::UPtr m_ik_solver;
  IVelocityProfiler::UPtr m_profiler;
  Transform m_tcp_frame;
  Transform m_base_frame;
  JointValues m_joints_state;
  JointValues m_joints_goal;
  Transform m_tcp_goal;
  std::function<void()> m_control_algorithm;
};

}  // namespace assignment