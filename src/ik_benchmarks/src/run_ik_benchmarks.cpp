#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <limits.h>
#include <random>
#include <chrono>
#include <numeric>

using namespace std::chrono_literals;

struct JointBounds {
  double min_position;
  double max_position;

  JointBounds()
      : min_position(std::numeric_limits<double>::min()),
        max_position(std::numeric_limits<double>::max()) {}
};

int main(int argc, char *argv[]) {

  // Initialize the node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ik_benchmarking_node", node_options);

  auto const logger = rclcpp::get_logger("ik_benchmarking");

  // Load the robot model, kinematic state, and joint model group
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr &robot_model =
      robot_model_loader.getModel();

  moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(robot_model));
  kinematic_state->setToDefaultValues();

  const moveit::core::JointModelGroup *joint_model_group =
      robot_model->getJointModelGroup("panda_arm");

  // Get joint names and bounds
  const std::vector<std::string> &joint_names =
      joint_model_group->getVariableNames();

  std::vector<JointBounds> joint_bounds(joint_model_group->getVariableCount());

  for (size_t i = 0; i < joint_model_group->getVariableCount(); ++i) {
    auto const &name = joint_names.at(i);
    auto const &bounds = robot_model->getVariableBounds(name);

    bool bounded = bounds.position_bounded_;

    if (bounded) {
      fmt::print("Joint {} has bounds of {} and {}\n", i + 1,
                 bounds.min_position_, bounds.max_position_);
      joint_bounds.at(i).min_position = bounds.min_position_;
      joint_bounds.at(i).max_position = bounds.max_position_;
    } else {
      fmt::print("Joints are unbounded!\n");
      // TODO: Handle this case. Should we assume a range? 
      return -1;
    }
  }

  // Collect IK solving data
  const int sample_size{1000};
  double success_count{0.0};
  std::vector<int> solve_times; //milliseconds
  
  // Sample random joints
  std::random_device rd;
  std::mt19937 generator(rd());

  for (size_t i = 0; i < sample_size; ++i) {
    std::vector<double> joint_values;

    for (const auto &bound : joint_bounds) {
      std::uniform_real_distribution<> distribution(bound.min_position,
                                                    bound.max_position);
      joint_values.push_back(distribution(generator));
    }
    fmt::print("Random joint values are:\n{}\n", joint_values);

    // Solve Forward Kinematics
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    kinematic_state->updateLinkTransforms();

    // TODO: Use the loaded tip here
    const Eigen::Isometry3d &end_effector_pose =
        kinematic_state->getGlobalLinkTransform("panda_link8");

    // Solve Inverse kinematics
    const auto start_time = std::chrono::high_resolution_clock::now();

    bool found_ik =
        kinematic_state->setFromIK(joint_model_group, end_effector_pose, 0.1);

    const auto end_time = std::chrono::high_resolution_clock::now(); 

    if (found_ik) {
      success_count++;
      const auto solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      solve_times.push_back(solve_time.count());      
    }
  }

  // Average IK solving time 
  double average_solve_time = std::accumulate(solve_times.begin(), solve_times.end(), 0.0)/solve_times.size();

  fmt::print("Success rate = {} and average IK solving time is {} ms\n", success_count/sample_size, average_solve_time);

  rclcpp::shutdown();
  return 0;
}