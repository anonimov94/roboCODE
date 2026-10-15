// KRL LIN milling node (port of ros_milling_cpp_full.cpp, ClaBirk/ROS2_Gazebo_Moveit_Simulation, GPL-3.0).
//
// Reads a KUKA KRL program (e.g. Fusion 360 CAM output, *.src) and executes it in the simulated cell
// with MoveIt 2 + Pilz: PTP to a start point above the workpiece, every "LIN {X..,Y..,Z..}" as a
// Pilz LIN of the spindle tip, "$VEL.CP=v" [m/s] as velocity scaling (Pilz max_trans_vel = 1 m/s),
// then PTP back to the start point.
//
// Changes vs. the original:
//  * planning group "kr10_arm" ends at the spindle tip (tcp) -> no hand-made link_6 -> tool offsets;
//  * targets are given in base_link + workpiece offset (no hard-coded world -> base transform);
//  * file and offsets are ROS parameters (the original had /home/claudius/... compiled in);
//  * robust parsing of "LIN {X 1,Y 2,Z 3,...}" and of the first line counting pass;
//  * a spinning executor so MoveIt sees the current robot state.
//
//   ros2 run robocode_milling_cpp krl_lin_milling --ros-args
//     -p krl_file:=$(ros2 pkg prefix robocode_milling_cpp)/share/robocode_milling_cpp/nc_program/sPlanen2.src
//
// Simulation only. Orientation: the spindle points down (KRL A/B/C are ignored, as in the original).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif

namespace
{
struct KrlCommand
{
  enum class Type { Lin, VelCp } type;
  double x = 0, y = 0, z = 0;  // mm
  double vel = 0;              // m/s
  int line = 0;
};

std::vector<KrlCommand> parseKrl(const std::string & filename, const rclcpp::Logger & logger)
{
  std::ifstream file(filename);
  std::vector<KrlCommand> out;
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Cannot open KRL file %s", filename.c_str());
    return out;
  }
  const std::string num = R"(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))";
  const std::regex lin_re("^\\s*LIN\\s*\\{\\s*X\\s+" + num + "\\s*,\\s*Y\\s+" + num + "\\s*,\\s*Z\\s+" + num,
    std::regex::icase);
  const std::regex vel_re("^\\s*\\$VEL\\.CP\\s*=\\s*" + num, std::regex::icase);
  std::string text;
  int line_no = 0;
  std::smatch m;
  while (std::getline(file, text)) {
    ++line_no;
    if (std::regex_search(text, m, lin_re)) {
      KrlCommand c{KrlCommand::Type::Lin};
      c.x = std::stod(m[1]);
      c.y = std::stod(m[2]);
      c.z = std::stod(m[3]);
      c.line = line_no;
      out.push_back(c);
    } else if (std::regex_search(text, m, vel_re)) {
      KrlCommand c{KrlCommand::Type::VelCp};
      c.vel = std::stod(m[1]);
      c.line = line_no;
      out.push_back(c);
    }
  }
  return out;
}

bool planAndExecute(
  moveit::planning_interface::MoveGroupInterface & mgi, const std::string & planner,
  const geometry_msgs::msg::PoseStamped & target, double vel_scale, double acc_scale,
  const rclcpp::Logger & logger)
{
  mgi.setPlannerId(planner);
  mgi.setPoseTarget(target);
  mgi.setMaxVelocityScalingFactor(vel_scale);
  mgi.setMaxAccelerationScalingFactor(acc_scale);
  mgi.setNumPlanningAttempts(10);
  mgi.setGoalTolerance(planner == "LIN" ? 0.001 : 0.01);  // 1 mm for LIN

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(mgi.plan(plan))) {
    RCLCPP_ERROR(logger, "%s planning failed", planner.c_str());
    return false;
  }
  return static_cast<bool>(mgi.execute(plan));
}

geometry_msgs::msg::PoseStamped toolDownPose(const std::string & frame, double x_mm, double y_mm, double z_mm)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame;
  // tcp frame: Z into the material -> rotated 180 deg about X relative to base_link
  msg.pose.orientation.x = 1.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 0.0;
  msg.pose.position.x = x_mm / 1000.0;
  msg.pose.position.y = y_mm / 1000.0;
  msg.pose.position.z = z_mm / 1000.0;
  return msg;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "krl_lin_milling", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  const auto group = node->get_parameter_or<std::string>("planning_group", "kr10_arm");
  const auto frame = node->get_parameter_or<std::string>("frame", "base_link");
  const auto krl_file = node->get_parameter_or<std::string>("krl_file", "");
  // workpiece zero in base_link [mm] (Wobj_*_base_mm of the original)
  const double wx = node->get_parameter_or<double>("workpiece_x_mm", 500.0);
  const double wy = node->get_parameter_or<double>("workpiece_y_mm", 250.0);
  const double wz = node->get_parameter_or<double>("workpiece_z_mm", 300.0);
  // start point relative to the workpiece zero [mm]
  const double sx = node->get_parameter_or<double>("start_x_mm", 75.0);
  const double sy = node->get_parameter_or<double>("start_y_mm", 75.0);
  const double sz = node->get_parameter_or<double>("start_z_mm", 150.0);
  const double acc = node->get_parameter_or<double>("acceleration_scaling", 0.1);

  int exit_code = 0;
  if (krl_file.empty()) {
    RCLCPP_ERROR(logger, "Set the parameter krl_file (KUKA *.src)");
    exit_code = 2;
  } else {
    const auto commands = parseKrl(krl_file, logger);
    size_t n_lin = 0;
    for (const auto & c : commands) {
      n_lin += c.type == KrlCommand::Type::Lin;
    }
    RCLCPP_INFO(logger, "%s: %zu LIN moves", krl_file.c_str(), n_lin);

    moveit::planning_interface::MoveGroupInterface mgi(node, group);
    mgi.setPlanningPipelineId("pilz_industrial_motion_planner");
    RCLCPP_INFO(logger, "Planning frame %s, end effector %s", mgi.getPlanningFrame().c_str(),
      mgi.getEndEffectorLink().c_str());

    const auto start = toolDownPose(frame, wx + sx, wy + sy, wz + sz);
    if (n_lin == 0 || !planAndExecute(mgi, "PTP", start, 0.1, 0.1, logger)) {
      exit_code = 1;
    } else {
      double vel = 0.1;
      size_t done = 0;
      for (const auto & c : commands) {
        if (c.type == KrlCommand::Type::VelCp) {
          vel = std::clamp(c.vel, 1e-4, 1.0);
          RCLCPP_INFO(logger, "line %d: $VEL.CP = %.4f m/s", c.line, vel);
          continue;
        }
        ++done;
        RCLCPP_INFO(logger, "line %d: LIN X %.3f Y %.3f Z %.3f (%zu/%zu)", c.line, c.x, c.y, c.z, done, n_lin);
        if (!planAndExecute(mgi, "LIN", toolDownPose(frame, wx + c.x, wy + c.y, wz + c.z), vel, acc, logger)) {
          RCLCPP_ERROR(logger, "Stopped at KRL line %d", c.line);
          exit_code = 1;
          break;
        }
      }
      planAndExecute(mgi, "PTP", start, 0.1, 0.1, logger);
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return exit_code;
}
