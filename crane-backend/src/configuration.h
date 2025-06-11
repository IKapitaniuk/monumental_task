#pragma once
#include <robot_description.h>

#include <string>
#include <vector>

namespace assignment {

struct Configuration {
  std::string json_config;
  std::string robot_name;
  std::vector<JointDescription> joints;
  TCPDescription tcp;
  BaseDescription base;
  IKSolverDescription ik;
};

}  // namespace assignment