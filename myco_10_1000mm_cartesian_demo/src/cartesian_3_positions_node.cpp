#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("myco_10_1000mm_cartesian_demo_node", options);
  auto logger = node->get_logger();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  constexpr char kPlanningGroup[] = "myco_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);

  RCLCPP_INFO(
    logger,
    "Planning frame: %s | End-effector link: %s",
    move_group.getPlanningFrame().c_str(),
    move_group.getEndEffectorLink().c_str());

  move_group.startStateMonitor(2.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setStartStateToCurrentState();

  node->declare_parameter<bool>("start_demo", false);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  auto run_demo = [&move_group, logger]() -> bool {
    // Move to a known state first so the Cartesian demo is repeatable.
    move_group.setNamedTarget("home");
    const auto home_result = move_group.move();
    if (home_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Failed to move to named target 'home'.");
      return false;
    }

    move_group.setStartStateToCurrentState();

    // Move to a non-singular posture before computing Cartesian path.
    {
      std::vector<double> pre_cartesian_joints = {0.0, -1.0, 1.2, 0.0, 0.4, 0.0};
      move_group.setJointValueTarget(pre_cartesian_joints);
      const auto pre_result = move_group.move();
      if (pre_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(logger, "Failed to move to pre-cartesian joint posture; using current posture.");
      }
      move_group.setStartStateToCurrentState();
    }

    const geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    auto build_waypoints = [&start_pose](double step) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve(3);

      geometry_msgs::msg::Pose pose_1 = start_pose;
      pose_1.position.z += step;
      waypoints.push_back(pose_1);

      geometry_msgs::msg::Pose pose_2 = pose_1;
      pose_2.position.y += step;
      waypoints.push_back(pose_2);

      geometry_msgs::msg::Pose pose_3 = pose_2;
      pose_3.position.x -= step;
      waypoints.push_back(pose_3);

      return waypoints;
    };

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;
    const bool avoid_collisions = true;

    double fraction = 0.0;
    {
      const auto waypoints = build_waypoints(0.02);
      fraction = move_group.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
    }

    if (fraction < 0.99) {
      RCLCPP_WARN(logger, "First Cartesian attempt reached %.2f%%. Retrying with smaller waypoints.", fraction * 100.0);
      const auto waypoints = build_waypoints(0.01);
      fraction = move_group.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
    }

    if (fraction < 0.99) {
      RCLCPP_WARN(
        logger,
        "Second attempt reached %.2f%%. Retrying with collision checks disabled (simulation fallback).",
        fraction * 100.0);
      const auto waypoints = build_waypoints(0.01);
      fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    }

    RCLCPP_INFO(logger, "Cartesian path completion: %.2f%%", fraction * 100.0);

    if (fraction < 0.99) {
      RCLCPP_ERROR(logger, "Cartesian path is incomplete; skipping execution.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    const auto result = move_group.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execution failed.");
      return false;
    }

    RCLCPP_INFO(logger, "Cartesian demo completed successfully.");
    return true;
  };

  RCLCPP_INFO(
    logger,
    "Waiting for parameter 'start_demo:=true' to run the Cartesian demo.");

  rclcpp::Rate loop_rate(5.0);
  while (rclcpp::ok()) {
    const bool start_demo = node->get_parameter("start_demo").as_bool();
    if (start_demo) {
      // Reset trigger immediately to enable edge-trigger behavior.
      node->set_parameter(rclcpp::Parameter("start_demo", false));
      RCLCPP_INFO(logger, "Received start_demo=true. Running Cartesian demo.");
      run_demo();
      RCLCPP_INFO(logger, "Demo cycle finished. Waiting for next start_demo trigger.");
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
