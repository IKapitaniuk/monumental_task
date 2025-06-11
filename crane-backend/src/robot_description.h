#pragma once

#include <eigen3/Eigen/Dense>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace assignment {

const std::unordered_map<std::string, unsigned> joint_to_index = {
    {"swing", 0},
    {"lift", 1},
    {"elbow", 2},
    {"wrist", 3},
};

const std::unordered_map<unsigned, std::string> index_to_joint = {
    {0, "swing"},
    {1, "lift"},
    {2, "elbow"},
    {3, "wrist"},
};

using JointState = std::unordered_map<std::string, double>;

using Transform = Eigen::Isometry3d;
using TransformMap =
    std::vector<std::tuple<std::string, Transform>>;  // + Add TCP frame
using Axis = Eigen::Vector3d;
using JointValues = std::vector<double>;
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

enum class JointType { Revolute, Prismatic };

struct JointDescription {
  std::string name;
  JointType type;
  Transform fixed_transform;
  Axis joint_axis;
  double limit_lower;
  double limit_upper;
  double max_velocity;
  double max_acceleration;
};

struct TCPDescription {
  std::string name;
  Transform fixed_transform;
};

struct BaseDescription {
  std::string name;
  double x;
  double y;
  double z;
  double yaw;
};

struct IKSolverDescription {
  Eigen::Vector3d lift_offset;
  Eigen::Vector3d elbow_offset;
  Eigen::Vector3d wrist_offset;
};

}  // namespace assignment