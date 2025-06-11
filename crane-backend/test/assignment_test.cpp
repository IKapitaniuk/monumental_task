#include <gmock/gmock.h>
#include <json_utils.h>
#include <kinematics.h>
#include <robot_controller.h>
#include <robot_description.h>
#include <velocity_profiler.h>

#include <cmath>
#include <memory>
#include <string>

using namespace assignment;
using ::testing::InSequence;
using ::testing::Return;
namespace pt = boost::property_tree;

TEST(Euler_Angles, EulerAnglesConversion) {
  Eigen::Vector3d rpy_in = {0.1, 0.2, 0.3};
  Eigen::Quaterniond q = euler_to_quaternion(rpy_in);
  Eigen::Vector3d rpy_out = q.toRotationMatrix().eulerAngles(0, 1, 2);
  for (unsigned idx = 0; idx < 3; ++idx) {
    ASSERT_DOUBLE_EQ(rpy_in[idx], rpy_out[idx]);
  }
}

TEST(JSON_Parsing, ReadFromString) {
  const std::string msg =
      "{ \"id\" : \"123\", \"number\" : \"456\", \"stuff\" : [{ \"name\" : "
      "\"test\" }, { \"name\" : \"some\" }, { \"name\" : \"stuffs\" }] }";
  auto root = read_json(msg);

  ASSERT_EQ(root.get<int>("id"), 123);
  ASSERT_EQ(root.get<std::string>("id"), "123");
  ASSERT_EQ(root.get<int>("number"), 456);
  ASSERT_EQ(root.get<std::string>("number"), "456");
  ASSERT_EQ(root.get_child("stuff").size(), 3);
  auto it = root.get_child("stuff").begin();
  ASSERT_EQ(it->second.get<std::string>("name"), "test");
  ++it;
  ASSERT_EQ(it->second.get<std::string>("name"), "some");
  ++it;
  ASSERT_EQ(it->second.get<std::string>("name"), "stuffs");
}

TEST(Kinematics, JointDescription) {
  JointDescription j{"Joint1",
                     JointType::Revolute,
                     Eigen::Isometry3d::Identity(),
                     Eigen::Vector3d::UnitZ(),
                     -3,
                     3,
                     2,
                     3};
  ASSERT_EQ(j.name, "Joint1");
  ASSERT_EQ(j.type, JointType::Revolute);
  ASSERT_EQ(j.fixed_transform.linear(), Eigen::Isometry3d::Identity().linear());
  ASSERT_EQ(j.fixed_transform.translation(),
            Eigen::Isometry3d::Identity().translation());
  ASSERT_EQ(j.joint_axis, Eigen::Vector3d::UnitZ());
  ASSERT_DOUBLE_EQ(j.limit_lower, -3.0);
  ASSERT_DOUBLE_EQ(j.limit_upper, 3.0);
  ASSERT_DOUBLE_EQ(j.max_velocity, 2);
  ASSERT_DOUBLE_EQ(j.max_acceleration, 3);

  Transform t = Eigen::Isometry3d::Identity();
  Eigen::Vector3d v = Eigen::Vector3d{1, 0, 0};
  t.translate(v);
  JointDescription j2{
      "Joint2", JointType::Prismatic, t, Eigen::Vector3d::UnitZ(), 0.0, 10.0, 2,
      3};
  ASSERT_EQ(j2.name, "Joint2");
  ASSERT_EQ(j2.type, JointType::Prismatic);
  ASSERT_EQ(j2.fixed_transform.linear(),
            Eigen::Isometry3d::Identity().linear());

  Eigen::Vector3d v2 = j2.fixed_transform.translation();
  ASSERT_EQ(v, v2);
  ASSERT_EQ(j2.joint_axis, Eigen::Vector3d::UnitZ());
  ASSERT_DOUBLE_EQ(j2.limit_lower, 0.0);
  ASSERT_DOUBLE_EQ(j2.limit_upper, 10.0);
  ASSERT_DOUBLE_EQ(j2.max_velocity, 2);
  ASSERT_DOUBLE_EQ(j2.max_acceleration, 3);
}

TEST(Kinematics, Kinematics2R00) {
  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});
  Transform t = Eigen::Isometry3d::Identity();
  t.translate(Eigen::Vector3d{1, 0, 0});
  parameters.push_back({"Joint2", JointType::Revolute, t,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  auto kin = ForwardKinematicsSolver(parameters);
  auto fk = kin.calc_forward_kinematics({0, 0});
  ASSERT_EQ(fk.size(), 4);

  {
    auto [name, tf] = fk[0];
    ASSERT_EQ(name, "base");
    ASSERT_EQ(tf.linear(), Eigen::Isometry3d::Identity().linear());
    ASSERT_EQ(tf.translation(), Eigen::Isometry3d::Identity().translation());
  }
  {
    auto [name, tf] = fk[1];
    ASSERT_EQ(name, "Joint1");
    ASSERT_EQ(tf.linear(), Eigen::Isometry3d::Identity().linear());
    ASSERT_EQ(tf.translation(), Eigen::Isometry3d::Identity().translation());
  }
  {
    auto [name, tf] = fk[2];
    ASSERT_EQ(name, "Joint2");
    ASSERT_EQ(tf.linear(), Eigen::Isometry3d::Identity().linear());
    ASSERT_TRUE(tf.translation() == Eigen::Vector3d(1, 0, 0));
  }
  {
    auto [name, tf] = fk[3];
    ASSERT_EQ(name, "tcp");
    ASSERT_EQ(tf.linear(), Eigen::Isometry3d::Identity().linear());
    ASSERT_TRUE(tf.translation() == Eigen::Vector3d(1, 0, 0));
  }
}

TEST(Kinematics, Kinematics2RDiffAngles) {
  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});

  Eigen::Vector3d v = Eigen::Vector3d{1, 2, 3};
  Transform t = Eigen::Isometry3d::Identity();
  t.translate(v);
  parameters.push_back({"Joint2", JointType::Revolute, t,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  Transform tcp = Eigen::Isometry3d::Identity();
  tcp.translate(v);

  auto kin = ForwardKinematicsSolver(parameters);
  kin.set_tcp_frame(tcp);
  double q1 = M_PI / 6;
  double q2 = M_PI / 4;
  auto fk = kin.calc_forward_kinematics({q1, q2});
  ASSERT_EQ(fk.size(), 4);

  {
    auto [name, tf] = fk[0];
    ASSERT_EQ(name, "base");
    ASSERT_EQ(tf.linear(), Eigen::Isometry3d::Identity().linear());
    ASSERT_EQ(tf.translation(), Eigen::Isometry3d::Identity().translation());
  }
  Eigen::Matrix3d R1 = Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()).matrix();
  {
    auto [name, tf] = fk[1];
    ASSERT_EQ(name, "Joint1");
    ASSERT_EQ(tf.linear(), R1);
    ASSERT_EQ(tf.translation(), Eigen::Isometry3d::Identity().translation());
  }
  Eigen::Matrix3d R2 =
      Eigen::AngleAxisd(q1 + q2, Eigen::Vector3d::UnitZ()).matrix();
  {
    auto [name, tf] = fk[2];
    ASSERT_EQ(name, "Joint2");
    ASSERT_DOUBLE_EQ((tf.linear() - R2).maxCoeff(), 0.0);
    ASSERT_TRUE(tf.translation() == R1 * v);
  }
  {
    auto [name, tf] = fk[3];
    ASSERT_EQ(name, "tcp");
    ASSERT_DOUBLE_EQ((tf.linear() - R2).maxCoeff(), 0.0);
    ASSERT_TRUE(tf.translation() == R2 * v + R1 * v);
  }
}

TEST(Kinematics, Kinematics2RBase) {
  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});

  Eigen::Vector3d v = Eigen::Vector3d{1, 2, 3};
  Transform t = Eigen::Isometry3d::Identity();
  t.translate(v);
  parameters.push_back({"Joint2", JointType::Revolute, t,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  Transform tcp = Eigen::Isometry3d::Identity();
  tcp.translate(v);

  Eigen::Vector3d base_pos = Eigen::Vector3d{4, 5, 1};
  double base_yaw = M_PI / 4;
  Transform base = Eigen::Isometry3d::Identity();
  base.translate(base_pos).rotate(
      Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ()));

  auto kin = ForwardKinematicsSolver(parameters);
  kin.set_tcp_frame(tcp);
  kin.set_base_frame(base);
  double q1 = M_PI / 3;
  double q2 = M_PI / 6;
  auto fk = kin.calc_forward_kinematics({q1, q2});
  ASSERT_EQ(fk.size(), 4);

  Eigen::Matrix3d R0 =
      Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ()).matrix();
  {
    auto [name, tf] = fk[0];
    ASSERT_EQ(name, "base");
    ASSERT_EQ(tf.linear(), R0);
    ASSERT_EQ(tf.translation(), base_pos);
  }
  Eigen::Matrix3d R1 =
      Eigen::AngleAxisd(base_yaw + q1, Eigen::Vector3d::UnitZ()).matrix();
  {
    auto [name, tf] = fk[1];
    ASSERT_EQ(name, "Joint1");
    EXPECT_NEAR((tf.linear() - R1).maxCoeff(), 0.0, 1e-15);
    ASSERT_EQ(tf.translation(), base_pos);
  }
  Eigen::Matrix3d R2 =
      Eigen::AngleAxisd(base_yaw + q1 + q2, Eigen::Vector3d::UnitZ()).matrix();
  {
    auto [name, tf] = fk[2];
    ASSERT_EQ(name, "Joint2");
    EXPECT_NEAR((tf.linear() - R2).maxCoeff(), 0.0, 1e-15);
    EXPECT_NEAR((tf.translation() - (R1 * v + base_pos)).maxCoeff(), 0.0,
                1e-15);
  }
  {
    auto [name, tf] = fk[3];
    ASSERT_EQ(name, "tcp");
    EXPECT_NEAR((tf.linear() - R2).maxCoeff(), 0.0, 1e-15);
    EXPECT_NEAR((tf.translation() - (R2 * v + R1 * v + base_pos)).maxCoeff(),
                0.0, 1e-15);
  }
}

TEST(Kinematics, KinematicsRP) {
  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});

  Eigen::Vector3d v = Eigen::Vector3d{1, 2, 3};
  Transform t = Eigen::Isometry3d::Identity();
  t.translate(v);
  parameters.push_back({"Joint2", JointType::Prismatic, t,
                        Eigen::Vector3d::UnitX(), -3, 3, 2, 3});

  Transform tcp = Eigen::Isometry3d::Identity();
  tcp.translate(v);

  auto kin = ForwardKinematicsSolver(parameters);
  kin.set_tcp_frame(tcp);

  double q1 = M_PI / 3;
  double q2 = 2;
  auto fk = kin.calc_forward_kinematics({q1, q2});
  ASSERT_EQ(fk.size(), 4);

  Eigen::Matrix3d R1 = Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()).matrix();
  {
    auto [name, tf] = fk[1];
    ASSERT_EQ(name, "Joint1");
    EXPECT_NEAR((tf.linear() - R1).maxCoeff(), 0.0, 1e-15);
    ASSERT_EQ(tf.translation(), Eigen::Vector3d::Zero());
  }
  Eigen::Vector3d v2 = Eigen::Vector3d::UnitX() * q2;
  {
    auto [name, tf] = fk[2];
    ASSERT_EQ(name, "Joint2");
    EXPECT_NEAR((tf.linear() - R1).maxCoeff(), 0.0, 1e-15);
    EXPECT_NEAR((tf.translation() - (R1 * (v + v2))).maxCoeff(), 0.0, 1e-15);
  }
  {
    auto [name, tf] = fk[3];
    ASSERT_EQ(name, "tcp");
    EXPECT_NEAR((tf.linear() - R1).maxCoeff(), 0.0, 1e-15);
    EXPECT_NEAR((tf.translation() - (R1 * v + R1 * (v + v2))).maxCoeff(), 0.0,
                1e-15);
  }
}

TEST(Kinematics, Jacobian2R) {
  double l1 = 1.0;
  double l2 = 2.0;

  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});
  Transform t = Eigen::Isometry3d::Identity();
  t.translate(Eigen::Vector3d{l1, 0, 0});
  parameters.push_back({"Joint2", JointType::Revolute, t,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  auto kin = ForwardKinematicsSolver(parameters);
  Transform t_ee = Eigen::Isometry3d::Identity();
  t_ee.translate(Eigen::Vector3d{l2, 0, 0});
  kin.set_tcp_frame(t_ee);

  double q1 = M_PI / 3;
  double q2 = M_PI / 4;
  Jacobian jac = kin.calc_jacobian({q1, q2});
  ASSERT_EQ(jac.rows(), 6);
  ASSERT_EQ(jac.cols(), 2);

  ASSERT_EQ(jac.block(3, 0, 3, 1), Eigen::Vector3d(0, 0, 1));
  ASSERT_EQ(jac.block(3, 1, 3, 1), Eigen::Vector3d(0, 0, 1));

  double s1 = sin(q1);
  double c1 = cos(q1);
  double s12 = sin(q1 + q2);
  double c12 = cos(q1 + q2);
  double a = -l1 * s1 - l2 * s12;
  double b = l1 * c1 + l2 * c12;
  double c = -l2 * s12;
  double d = l2 * c12;

  EXPECT_NEAR((jac.block(0, 0, 3, 1) - Eigen::Vector3d(a, b, 0)).maxCoeff(),
              0.0, 1e-15);
  EXPECT_NEAR((jac.block(0, 1, 3, 1) - Eigen::Vector3d(c, d, 0)).maxCoeff(),
              0.0, 1e-15);
}

TEST(Kinematics, JacobianRP) {
  double l1 = 1.0;
  double l2 = 2.0;

  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});

  Transform t = Eigen::Isometry3d::Identity();
  t.translate(Eigen::Vector3d{l1, 0, 0});
  parameters.push_back({"Joint2", JointType::Prismatic, t,
                        Eigen::Vector3d::UnitX(), -3, 3, 2, 3});

  auto kin = ForwardKinematicsSolver(parameters);
  Transform t_ee = Eigen::Isometry3d::Identity();
  t_ee.translate(Eigen::Vector3d{l2, 0, 0});
  kin.set_tcp_frame(t_ee);

  double q1 = M_PI / 3;
  double q2 = 0.5;
  Jacobian jac = kin.calc_jacobian({q1, q2});
  ASSERT_EQ(jac.rows(), 6);
  ASSERT_EQ(jac.cols(), 2);

  ASSERT_EQ(jac.block(3, 0, 3, 1), Eigen::Vector3d(0, 0, 1));
  ASSERT_EQ(jac.block(3, 1, 3, 1), Eigen::Vector3d(0, 0, 0));

  double s1 = sin(q1);
  double c1 = cos(q1);

  double a = -s1 * (l1 + l2 + q2);
  double b = c1 * (l1 + l2 + q2);
  double c = c1;
  double d = s1;

  EXPECT_NEAR((jac.block(0, 0, 3, 1) - Eigen::Vector3d(a, b, 0)).maxCoeff(),
              0.0, 1e-15);
  EXPECT_NEAR((jac.block(0, 1, 3, 1) - Eigen::Vector3d(c, d, 0)).maxCoeff(),
              0.0, 1e-15);
}

TEST(Kinematics, InverseKinematics) {
  Eigen::Vector3d lift_offset(0, 0, 0.3);
  Eigen::Vector3d elbow_offset(0.5, 0.1, -0.1);
  Eigen::Vector3d wrist_offset(0.3, 0.1, -0.2);
  IKSolverDescription ik_parameters{lift_offset, elbow_offset, wrist_offset};

  std::vector<JointDescription> parameters;
  parameters.push_back({"Joint1", JointType::Revolute,
                        Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitZ(),
                        -3, 3, 2, 3});

  Transform t_lift = Eigen::Isometry3d::Identity();
  t_lift.translate(lift_offset);
  parameters.push_back({"Lift", JointType::Prismatic, t_lift,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  Transform t_elbow = Eigen::Isometry3d::Identity();
  t_elbow.translate(elbow_offset);
  parameters.push_back({"Elbow", JointType::Revolute, t_elbow,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  Transform t_wrist = Eigen::Isometry3d::Identity();
  t_wrist.translate(wrist_offset);
  parameters.push_back({"Wrist", JointType::Revolute, t_wrist,
                        Eigen::Vector3d::UnitZ(), -3, 3, 2, 3});

  auto fk = ForwardKinematicsSolver(parameters);
  auto ik = InverseKinematicsSolver4DOF(ik_parameters, parameters);

  Eigen::Vector3d desired_position(0.8, 0.1, 1);
  double desired_yaw = M_PI / 2;
  Transform desired_pose = Eigen::Isometry3d::Identity();
  desired_pose.translate(desired_position)
      .rotate(Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ()));

  auto ik_solutions = ik.calc_inverse_kinematics(desired_pose);
  ASSERT_EQ(ik_solutions.size(), 2);

  auto fk1 = fk.calc_forward_kinematics(ik_solutions[0]);
  Eigen::Vector3d resulting_position_1 = std::get<1>(fk1.back()).translation();
  Eigen::Vector3d resulting_rpy_1 =
      std::get<1>(fk1.back()).rotation().eulerAngles(0, 1, 2);
  // std::cout << resulting_pose_1.translation() << std::endl;
  EXPECT_NEAR((resulting_position_1 - desired_position).maxCoeff(), 0.0, 1e-15);
  EXPECT_NEAR(resulting_rpy_1[2] - desired_yaw, 0.0, 1e-15);

  auto fk2 = fk.calc_forward_kinematics(ik_solutions[1]);
  Eigen::Vector3d resulting_position_2 = std::get<1>(fk2.back()).translation();
  Eigen::Vector3d resulting_rpy_2 =
      std::get<1>(fk2.back()).rotation().eulerAngles(0, 1, 2);
  // std::cout << resulting_pose_1.translation() << std::endl;
  EXPECT_NEAR((resulting_position_2 - desired_position).maxCoeff(), 0.0, 1e-15);
  EXPECT_NEAR(resulting_rpy_2[2] - desired_yaw, 0.0, 1e-15);
}

class MockProfiler : public IVelocityProfiler {
 public:
  MOCK_METHOD(void, set_state, (const JointValues& setpoints), (override));
  MOCK_METHOD(void, set_target, (const JointValues& setpoints), (override));
  MOCK_METHOD(JointValues, update, (), (override));
};

class MockFKSolver : public IForwardKinematicsSolver {
 public:
  MOCK_METHOD(void, set_tcp_frame, (const Transform&), (override));
  MOCK_METHOD(void, set_base_frame, (const Transform&), (override));
  MOCK_METHOD(TransformMap, calc_forward_kinematics,
              (const JointValues& joint_values), (const, override));
  MOCK_METHOD(Jacobian, calc_jacobian, (const JointValues& joint_values),
              (const, override));
};

class MockIKSolver : public IInverseKinematicsSolver {
 public:
  MOCK_METHOD(std::vector<JointValues>, calc_inverse_kinematics,
              (const Transform&), (const, override));
};

TEST(RobotController, SetFrames) {
  auto fk_solver = std::make_unique<MockFKSolver>();
  auto ik_solver = std::make_unique<MockIKSolver>();
  auto profiler = std::make_unique<MockProfiler>();
  {
    InSequence seq;
    using ::testing::_;
    EXPECT_CALL(*fk_solver, set_tcp_frame(_)).Times(1);
    EXPECT_CALL(*fk_solver, set_base_frame(_)).Times(1);
    EXPECT_CALL(*fk_solver, set_tcp_frame(_)).Times(1);
    EXPECT_CALL(*fk_solver, set_base_frame(_)).Times(1);
  }
  RobotController4DOF ctrl(0.1, std::move(fk_solver), std::move(ik_solver),
                           std::move(profiler));
  Transform tf = Eigen::Isometry3d::Identity();
  ctrl.set_tcp_frame(tf);
  ctrl.set_base_frame(tf);
}

TEST(RobotController, SetJointSetpoint) {
  auto fk_solver = std::make_unique<MockFKSolver>();
  auto ik_solver = std::make_unique<MockIKSolver>();
  auto profiler = std::make_unique<MockProfiler>();
  {
    InSequence seq;
    using ::testing::_;
    EXPECT_CALL(*fk_solver, set_tcp_frame(_)).Times(1);
    EXPECT_CALL(*fk_solver, set_base_frame(_)).Times(1);
    EXPECT_CALL(*profiler, set_state(_)).Times(1);
    EXPECT_CALL(*profiler, set_target(_)).Times(1);
    EXPECT_CALL(*profiler, update()).Times(1);
  }
  RobotController4DOF ctrl(0.1, std::move(fk_solver), std::move(ik_solver),
                           std::move(profiler));
  JointValues s;
  ctrl.set_joint_setpoint(s);
  ctrl.update();
}

TEST(RobotController, SetPoseSetpoint) {
  auto fk_solver = std::make_unique<MockFKSolver>();
  auto ik_solver = std::make_unique<MockIKSolver>();
  auto profiler = std::make_unique<MockProfiler>();

  {
    InSequence seq;
    using ::testing::_;
    EXPECT_CALL(*fk_solver, set_tcp_frame(_)).Times(1);
    EXPECT_CALL(*fk_solver, set_base_frame(_)).Times(1);
    EXPECT_CALL(*profiler, set_state(_)).Times(1);
    std::vector<JointValues> solutions(1);
    EXPECT_CALL(*ik_solver, calc_inverse_kinematics(_))
        .Times(1)
        .WillOnce(Return(solutions));
    EXPECT_CALL(*profiler, set_target(_)).Times(1);
    EXPECT_CALL(*profiler, update()).Times(1);
  }
  RobotController4DOF ctrl(0.1, std::move(fk_solver), std::move(ik_solver),
                           std::move(profiler));
  Transform tf = Eigen::Isometry3d::Identity();
  ctrl.set_pose_setpoint(tf);
  ctrl.update();
}
