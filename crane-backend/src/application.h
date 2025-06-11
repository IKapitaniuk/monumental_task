#pragma once
#include <configuration.h>
#include <json_utils.h>
#include <robot_controller_if.h>
#include <robot_sim.h>
#include <server_if.h>
#include <timer_if.h>

#include <boost/asio.hpp>
#include <memory>
#include <string>

namespace assignment {

class Application {
 public:
  Application(unsigned port, Configuration&& config);
  ~Application();
  void run();

 private:
  void on_control_cycle();
  void on_msg_cycle();
  void msg_dispatch(const std::string& msg);
  void set_joint_command(pt::ptree msg);
  void set_pose_command(pt::ptree msg);
  void send_config();

  boost::asio::io_context m_ioc;
  double m_dt;
  double m_t;
  Configuration m_config;
  std::unique_ptr<IServer<std::string>> m_server;
  std::unique_ptr<ITimer> m_msg_timer;
  std::unique_ptr<ITimer> m_control_timer;
  std::unique_ptr<IRobotInterface> m_robot;
  std::unique_ptr<IRobotController> m_controller;
};

}  // namespace assignment