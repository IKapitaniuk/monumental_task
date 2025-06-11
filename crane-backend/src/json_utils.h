#pragma once

#include <configuration.h>
#include <robot_description.h>

#include <filesystem>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#undef BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <fstream>
#include <sstream>
#include <string>

namespace assignment {

namespace pt = boost::property_tree;

Eigen::Quaterniond euler_to_quaternion(Eigen::Vector3d rpy);
Configuration read_configuration(const std::filesystem::path& path);

pt::ptree read_json(const std::string& msg);
std::string make_json(const pt::ptree& root);

pt::ptree joints_to_json(const JointValues& values);
JointValues json_to_joints(pt::ptree& root);

pt::ptree pose_to_json(const Transform& tf);
Transform json_to_pose(pt::ptree& root);

std::string make_state_msg(const JointValues& joints, const Transform& tcp_pose,
                           const Transform& base_pose);

std::string make_configuration_msg(const std::string& config);

}  // namespace assignment
