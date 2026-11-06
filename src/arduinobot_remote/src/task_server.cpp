#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <thread>


using namespace std::placeholders;

namespace arduinobot_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));

    this->declare_parameter<std::vector<double>>("table_size", {1.5, 1.5, 0.4});
    this->declare_parameter<std::vector<double>>("table_pose", {0.0, 1.5, 0.2});

    auto table_size = this->get_parameter("table_size").as_double_array();
    auto table_pose_values = this->get_parameter("table_pose").as_double_array();

    moveit_msgs::msg::CollisionObject table_object;
    table_object.header.frame_id = "base_link";
    table_object.id = "table";

    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions = {table_size[0], table_size[1], table_size[2]}; 

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = table_pose_values[0];
    table_pose.position.y = table_pose_values[1];
    table_pose.position.z = table_pose_values[2];
    table_pose.orientation.w = 1.0;

    table_object.primitives.push_back(table_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;

    planning_scene_interface_.applyCollisionObject(table_object);
  }

private:
  rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<double> arm_joint_goal_, gripper_joint_goal_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with task_number %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(arm_move_group_){
        arm_move_group_->stop();
    }
    if(gripper_move_group_){
        gripper_move_group_->stop();
    }
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  bool planAndMove(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group->move();
    }
    return success;
  }

  void executeDynamicPick(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    double x = goal_handle->get_goal()->target_x;
    double y = goal_handle->get_goal()->target_y;
    double z = goal_handle->get_goal()->target_z;

    std::vector<double> gripper_open = {-0.7, 0.7};
    std::vector<double> gripper_closed = {-0.102, 0.102};

    auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

    gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());
    gripper_move_group_->setJointValueTarget(gripper_open);
    if (!planAndMove(gripper_move_group_)) {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper");
      return;
    }

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    arm_move_group_->setPositionTarget(x, y, z + 0.10, "gripper_tcp");
    if (!planAndMove(arm_move_group_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach approach pose");
      return;
    }

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    arm_move_group_->setPositionTarget(x, y, z + 0.09, "gripper_tcp");
    if (!planAndMove(arm_move_group_)) {
      RCLCPP_ERROR(get_logger(), "Failed to descend to object");
      return;
    }

    gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());
    gripper_move_group_->setJointValueTarget(gripper_closed);
    if (!planAndMove(gripper_move_group_)) {
      RCLCPP_ERROR(get_logger(), "Failed to close gripper");
      return;
    }

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    arm_move_group_->setPositionTarget(x, y, z + 0.80, "gripper_tcp");
    if (!planAndMove(arm_move_group_)) {
      RCLCPP_ERROR(get_logger(), "Failed to lift object");
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    if(!arm_move_group_){
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    }
    if(!gripper_move_group_){
        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    }
    if (goal_handle->get_goal()->task_number == 3) {
        executeDynamicPick(goal_handle);
        return;
    }

    auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

    if(goal_handle->get_goal()->task_number == 0){
        arm_joint_goal_= {0.0, 0.0, 0.0};
        gripper_joint_goal_= {-0.7, 0.7};
    }else if (goal_handle->get_goal()->task_number == 1){
        arm_joint_goal_ = {-1.14, -0.6, -0.07};
        gripper_joint_goal_ = {0.0, 0.0};
    } else if(goal_handle->get_goal()->task_number == 2){
        arm_joint_goal_ = {-1.57, -0.0, -0.9};
        gripper_joint_goal_ = {0.0, 0.0};
    } else {
        RCLCPP_ERROR(get_logger(), "Invalide Task Number");
        return;
    }

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

    bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
    bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);
    
    if(!arm_within_bounds && !gripper_within_bounds){
        RCLCPP_ERROR(get_logger(), "Target position out of boundaries");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan;
    bool arm_plan_sucess = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_sucess = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm_plan_sucess && gripper_plan_sucess){
        arm_move_group_->move();
        gripper_move_group_->move();
    } else {
        RCLCPP_ERROR(get_logger(),"One or more planner failed");
        return;
    }

    result->success = true;
    goal_handle->succeed(result);

  }
};
}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)