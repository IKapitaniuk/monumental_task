#include <json_utils.h>
#include <robot_description.h>

#include <cassert>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace assignment {

namespace pt = boost::property_tree;

Eigen::Quaterniond euler_to_quaternion(Eigen::Vector3d rpy) {
  Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(rpy[2], Eigen::Vector3d::UnitZ());
  return rollAngle * pitchAngle * yawAngle;
}

Eigen::Vector3d read_vector3d(pt::ptree root) {
  Eigen::Vector3d v;
  auto it = root.begin();
  for (unsigned idx = 0; idx < 3; ++idx, ++it) {
    v[idx] = it->second.get_value<double>();
  }
  return v;
}

JointDescription read_joint_description(pt::ptree joint) {
  JointDescription desc;
  desc.name = joint.get<std::string>("name");
  std::string type = joint.get<std::string>("type");
  if (type == "revolute") {
    desc.type = JointType::Revolute;
  } else if (type == "prismatic") {
    desc.type = JointType::Prismatic;
  } else {
    throw(std::runtime_error("Unknown joint type\n"));
  }
  Axis axis = read_vector3d(joint.get_child("axis"));
  desc.joint_axis = axis.normalized();

  Eigen::Vector3d pos = read_vector3d(joint.get_child("position"));
  Eigen::Vector3d rpy = read_vector3d(joint.get_child("rotation_rpy"));

  Transform pose = Eigen::Isometry3d::Identity();
  pose.translation() = pos;
  pose.linear() = euler_to_quaternion(rpy).toRotationMatrix();
  desc.fixed_transform = pose;

  auto low = joint.get_optional<double>("lower_limit");
  auto up = joint.get_optional<double>("upper_limit");
  desc.limit_lower = low.get_value_or(std::numeric_limits<double>::lowest());
  desc.limit_upper = up.get_value_or(std::numeric_limits<double>::max());

  auto vel = joint.get_optional<double>("max_velocity");
  auto acc = joint.get_optional<double>("max_acceleration");
  desc.max_velocity = vel.get_value_or(0.0);
  desc.max_acceleration = acc.get_value_or(0.0);

  return desc;
}

TCPDescription read_tcp_description(pt::ptree tcp) {
  TCPDescription desc;
  auto name = tcp.get_optional<std::string>("name");
  desc.name = name.get_value_or("tcp");

  Eigen::Vector3d pos = read_vector3d(tcp.get_child("position"));
  Eigen::Vector3d rpy = read_vector3d(tcp.get_child("rotation_rpy"));

  Transform pose = Eigen::Isometry3d::Identity();
  pose.translation() = pos;
  pose.linear() = euler_to_quaternion(rpy).toRotationMatrix();
  desc.fixed_transform = pose;

  return desc;
}

BaseDescription read_base_description(pt::ptree root) {
  std::string name =
      root.get_optional<std::string>("name").get_value_or("base");

  Eigen::Vector3d pos = read_vector3d(root.get_child("position"));
  double yaw = root.get_optional<double>("yaw").value_or(0.0);

  return BaseDescription{name, pos.x(), pos.y(), pos.z(), yaw};
}

IKSolverDescription read_ik_description(pt::ptree ik) {
  IKSolverDescription desc;
  desc.lift_offset = read_vector3d(ik.get_child("lift_offset"));
  desc.elbow_offset = read_vector3d(ik.get_child("elbow_offset"));
  desc.wrist_offset = read_vector3d(ik.get_child("wrist_offset"));
  return desc;
}

Configuration read_configuration(const std::filesystem::path& path) {
  Configuration config;
  std::ifstream f(path, std::ios::in);
  const auto size = std::filesystem::file_size(path);
  std::string json_configuration(size, '\0');
  f.read(json_configuration.data(), size);
  config.json_config = std::move(json_configuration);
  pt::ptree root = read_json(config.json_config);
  auto robot_name = root.get_optional<std::string>("name");
  config.robot_name = robot_name.get_value_or("unknown");
  for (const auto& joint : root.get_child("joints")) {
    JointDescription desc = read_joint_description(joint.second);
    config.joints.push_back(desc);
  }
  config.tcp = read_tcp_description(root.get_child("tcp"));
  config.base = read_base_description(root.get_child("base"));
  config.ik = read_ik_description(root.get_child("ik"));

  return config;
}

pt::ptree read_json(const std::string& msg) {
  std::istringstream is(msg);
  pt::ptree root;
  pt::read_json(is, root);
  return root;
}

std::string make_json(const pt::ptree& root) {
  std::ostringstream os;
  pt::write_json(os, root);
  return os.str();
}

pt::ptree joints_to_json(const JointValues& values) {
  assert(values.size() == 4);
  pt::ptree joints;
  for (unsigned idx = 0; idx < values.size(); ++idx) {
    pt::ptree node;
    node.put("name", index_to_joint.at(idx));
    node.put<double>("value", values[idx]);
    joints.push_back({"", node});
  }
  return joints;
}

JointValues json_to_joints(pt::ptree& root) {
  JointValues state;
  auto n = root.get_child("joints").size();
  state.resize(n);
  for (pt::ptree::value_type& entry : root.get_child("joints")) {
    auto name = entry.second.get<std::string>("name");
    auto value = entry.second.get<double>("value");
    auto idx = joint_to_index.at(name);
    state[idx] = value;
  }
  return state;
}

pt::ptree pose_to_json(const Transform& tf) {
  pt::ptree pose;

  Eigen::Vector3d tf_position = tf.translation();
  pt::ptree position;
  position.put("x", tf_position.x());
  position.put("y", tf_position.y());
  position.put("z", tf_position.z());
  pose.add_child("position", position);

  Eigen::Quaterniond tf_quat{tf.rotation()};
  pt::ptree quaternion;
  quaternion.put("x", tf_quat.x());
  quaternion.put("y", tf_quat.y());
  quaternion.put("z", tf_quat.z());
  quaternion.put("w", tf_quat.w());
  pose.add_child("quaternion", quaternion);

  return pose;
}

Transform json_to_pose(pt::ptree& root) {
  auto x = root.get<double>("position.x");
  auto y = root.get<double>("position.y");
  auto z = root.get<double>("position.z");

  auto qx = root.get<double>("quaternion.x");
  auto qy = root.get<double>("quaternion.y");
  auto qz = root.get<double>("quaternion.z");
  auto qw = root.get<double>("quaternion.w");

  Transform pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d{x, y, z};
  Eigen::Quaterniond q{qw, qx, qy, qz};
  q.normalize();
  pose.linear() = q.toRotationMatrix();

  return pose;
}

std::string make_state_msg(const JointValues& joints, const Transform& tcp_pose,
                           const Transform& base_pose) {
  pt::ptree root;
  root.put("msg_id", "state");
  root.add_child("joints", joints_to_json(joints));
  root.add_child("tcp_pose", pose_to_json(tcp_pose));
  root.add_child("base_pose", pose_to_json(base_pose));
  return make_json(root);
}

std::string make_configuration_msg(const std::string& config) {
  pt::ptree root;
  root.put("msg_id", "configuration_update");
  root.put("configuration", config);
  return make_json(root);
}

}  // namespace assignment