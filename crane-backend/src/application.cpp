#include <application.h>
#include <json_utils.h>
#include <kinematics.h>
#include <robot_controller.h>
#include <robot_description.h>
#include <timer.h>
#include <velocity_profiler.h>
#include <ws_server.h>

#include <boost/asio.hpp>
#include <iostream>

namespace assignment {

Application::Application(unsigned port, Configuration&& config)
    : m_ioc{1}, m_dt{0.0}, m_t{0.0}, m_config{std::move(config)} {
  auto const address = boost::asio::ip::make_address("0.0.0.0");

  m_server = std::make_unique<WebsocketServer>(
      m_ioc, address, static_cast<unsigned short>(port));
  m_server->add_message_callback(
      [this](const std::string& msg) { this->msg_dispatch(msg); });

  m_dt = 0.001;  // in seconds - 1 millisecond control cycle
  boost::posix_time::millisec ctrl_interval(static_cast<long>(m_dt * 1000));
  boost::posix_time::millisec msg_interval(33);

  m_robot = std::make_unique<CraneSim4DOF>(m_config);

  m_controller = std::make_unique<RobotController4DOF>(
      m_dt, std::make_unique<ForwardKinematicsSolver>(m_config.joints),
      std::make_unique<InverseKinematicsSolver4DOF>(m_config.ik,
                                                    m_config.joints),
      std::make_unique<RuckigProfiler>(4, m_dt, m_config.joints));

  m_controller->set_tcp_frame(m_config.tcp.fixed_transform);
  m_controller->set_feedback_state(m_robot->get_feedback());
  m_controller->set_base_frame(m_robot->get_base_pose());

  m_control_timer = std::make_unique<Timer>(m_ioc, ctrl_interval);
  m_control_timer->add_callback([this]() { this->on_control_cycle(); });
  m_control_timer->start();

  m_msg_timer = std::make_unique<Timer>(m_ioc, msg_interval);
  m_msg_timer->add_callback([this]() { this->on_msg_cycle(); });
  m_msg_timer->start();
}

Application::~Application() {}

void Application::run() {
  m_server->run();
  m_ioc.run();
}

void Application::on_msg_cycle() {
  auto joint_fb = m_robot->get_feedback();
  auto tcp_pose = m_controller->get_tcp_pose();
  auto base_pose = m_robot->get_base_pose();
  std::string msg = make_state_msg(joint_fb, tcp_pose, base_pose);
  m_server->publish(msg);
}

void Application::on_control_cycle() {
  if (m_controller->enabled_stabilization()) {
    m_t += m_dt;  // in sec
    m_robot->set_base_command(0.2 * sin(0.1 * m_t), 0.6 * sin(0.2 * m_t),
                              0.1 * sin(0.3 * m_t), 1 * sin(0.1 * m_t));
  }

  m_robot->update(m_dt);
  // m_controller->set_feedback_state(m_robot->get_feedback());
  m_controller->set_base_frame(m_robot->get_base_pose());
  m_controller->update();
  m_robot->set_command(m_controller->get_joint_setpoint());
}

void Application::msg_dispatch(const std::string& msg) {
  pt::ptree root = read_json(msg);
  auto msg_id = root.get_optional<std::string>("msg_id");
  if (!msg_id) return;
  if (*msg_id == "joint_command") {
    set_joint_command(root);

  } else if (*msg_id == "pose_command") {
    set_pose_command(root);
  } else if (*msg_id == "configuration_request") {
    send_config();
  }
}

void Application::send_config() {
  std::string msg = make_configuration_msg(m_config.json_config);
  m_server->publish(msg);
}

void Application::set_joint_command(pt::ptree msg) {
  auto jnt_command = json_to_joints(msg);
  m_controller->enable_stabilization(false);
  m_controller->set_feedback_state(m_robot->get_feedback());
  m_controller->set_joint_setpoint(jnt_command);
}

void Application::set_pose_command(pt::ptree msg) {
  auto stab = msg.get_optional<bool>("stabilization");
  if (stab) {
    m_controller->enable_stabilization(*stab);
  }
  auto algo = msg.get_optional<std::string>("control_algorithm");
  if (algo) {
    if (*algo == "ik") {
      m_controller->set_control_algorithm(ControlAlgorithm::ik);
    } else if (*algo == "jacobian") {
      m_controller->set_control_algorithm(ControlAlgorithm::jacobian);
    }
  }
  auto p = msg.get_child_optional("tcp_pose");
  if (p) {
    auto pose_command = json_to_pose(*p);
    m_controller->set_feedback_state(m_robot->get_feedback());
    m_controller->set_pose_setpoint(pose_command);
  }
}

}  // namespace assignment