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

struct JointBounds
{
  double min_position;
  double max_position;

  JointBounds()
      : min_position(std::numeric_limits<double>::min()),
        max_position(std::numeric_limits<double>::max()) {}
};

int main(int argc, char *argv[])
{
  // Initialize the node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ik_benchmarking_node", node_options);

  auto const logger = rclcpp::get_logger("ik_benchmarking");

  // Load the robot model, kinematic state, and joint model group
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  robot_state->setToDefaultValues();

  std::string move_group_name = node->get_parameter("move_group").as_string();
  const moveit::core::JointModelGroup *joint_model_group = robot_model->getJointModelGroup(move_group_name);

  // Get joint names and bounds
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  std::vector<JointBounds> joint_bounds(joint_model_group->getVariableCount());

  // TODO(Mohamed): use only active joints/variables because getVariableCount returns all joint types
  for (size_t i = 0; i < joint_model_group->getVariableCount(); ++i)
  {
    auto const &name = joint_names.at(i);
    auto const &bounds = robot_model->getVariableBounds(name);

    bool bounded = bounds.position_bounded_;

    if (bounded)
    {
      fmt::print("Joint {} has bounds of {} and {}\n", i + 1,
                 bounds.min_position_, bounds.max_position_);
      joint_bounds.at(i).min_position = bounds.min_position_;
      joint_bounds.at(i).max_position = bounds.max_position_;
    }
    else
    {
      fmt::print("Joints are unbounded!\n");
      // TODO (Mohamed): Handle this case. Should we assume a range?
      return -1;
    }
  }

  // Load the tip link name (not the end effector)
  std::string tip_link_name{};
  auto const &link_names = joint_model_group->getLinkModelNames();

  if (!link_names.empty())
  {
    tip_link_name = link_names.back();
  }
  else
  {
    RCLCPP_ERROR(logger, "ERROR: The move group is corrupted. Links count is zero.\n");
    rclcpp::shutdown();
  }

  const Eigen::Isometry3d &tip_link_pose = robot_state->getGlobalLinkTransform(tip_link_name);

  // Collect IK solving data
  const int sample_size{1000};
  double success_count{0.0};
  std::vector<int> solve_times; // milliseconds

  // Sample random joints
  std::random_device rd;
  std::mt19937 generator(rd());

  for (size_t i = 0; i < sample_size; ++i)
  {
    std::vector<double> joint_values;

    for (const auto &bound : joint_bounds)
    {
      std::uniform_real_distribution<> distribution(bound.min_position,
                                                    bound.max_position);
      joint_values.push_back(distribution(generator));
    }
    fmt::print("Random joint values are:\n{}\n", joint_values);

    // Solve Forward Kinematics
    robot_state->setJointGroupPositions(joint_model_group, joint_values);
    robot_state->updateLinkTransforms();

    // Solve Inverse kinematics
    const auto start_time = std::chrono::high_resolution_clock::now();

    bool found_ik =
        robot_state->setFromIK(joint_model_group, tip_link_pose, 0.1);

    const auto end_time = std::chrono::high_resolution_clock::now();

    if (found_ik)
    {
      success_count++;
      const auto solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      solve_times.push_back(solve_time.count());
    }
  }

  // Average IK solving time
  double average_solve_time = std::accumulate(solve_times.begin(), solve_times.end(), 0.0) / solve_times.size();

  fmt::print("Success rate = {} and average IK solving time is {} ms\n", success_count / sample_size, average_solve_time);

  rclcpp::shutdown();
  return 0;
}