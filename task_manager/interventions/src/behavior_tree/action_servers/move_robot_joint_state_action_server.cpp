#include "interventions/behavior_tree/action_servers/move_robot_joint_state_action_server.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

MoveRobotJointStateActionServer::MoveRobotJointStateActionServer(const std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    //Lifelines for actions are specified per server and inside the namespace to prevent clashing
    //and correct function when the same action server is used in multiple interventions
    action_lifeline_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("move_robot_joint_state_action_lifeline", 1); 

    //Parameters
    node_->declare_parameter<int>("tree_frequency", 100);
    node_->declare_parameter<std::string>("action_name", "");
    node_->declare_parameter<bool>("print", true);
    node_->declare_parameter<std::string>("robot_name", "");
    node_->get_parameter<int>("tree_frequency", tree_frequency_);
    node_->get_parameter<std::string>("action_name", action_name_);
    node_->get_parameter<bool>("print", print_);
    node_->get_parameter<std::string>("robot_name", robot_name_);

    logger_ = std::make_shared<Logger>(print_);

    //Perform checks on input parameters
    if(action_name_ == "" || action_name_.empty())
    {
        logger_->logError(node_, "No valid action name specified! Exiting");
        return;
    }
    if(tree_frequency_ <= 0)
    {
        logger_->logError(node_, "No valid tree update frequency specified! Exiting");
        return;
    }
    if(robot_name_ == "" || robot_name_.empty())
    {
        logger_->logWarning(node_, "No robot name specified! Using default value 'manipulator'");
        robot_name_ = "manipulator";
    }

    //Initialise Moveit
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, robot_name_);

    //Print initialisation message
    std::stringstream ss;
    ss << "MOVE ROBOT JOINT STATE ACTION SERVER INITIALISATION\n";
    ss << "Action name: " << action_name_ << "\n";
    ss << "Robot name: " << robot_name_ << "\n";
    ss << "Tree update frequency: " << std::to_string(tree_frequency_) << " ms\n";
    logger_->logInfo(node_, ss);

    //Create the lifeline publisher timer
    action_lifeline_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_), std::bind(&MoveRobotJointStateActionServer::lifeline, this));

    //Create action server
    this->action_server_ = rclcpp_action::create_server<MoveAction>(
        node_, 
        action_name_,
        std::bind(&MoveRobotJointStateActionServer::handleGoal, this, _1, _2),
        std::bind(&MoveRobotJointStateActionServer::handleCancel, this, _1),
        std::bind(&MoveRobotJointStateActionServer::handleAccepted, this, _1));
}

rclcpp_action::GoalResponse MoveRobotJointStateActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveAction::Goal> goal)
{   
    //Check if the input is valid 
    //Check if the velocity scaling is a valid value between 0 and 1
    if((goal->vel_scaling <= 0.0) || (goal->vel_scaling > 1.0))
    {
        std::stringstream ss;
        ss << "Goal rejected as the velocity scaling factor: " << std::to_string(goal->vel_scaling) << " is not possible\n";
        logger_->logError(node_, ss);
        return rclcpp_action::GoalResponse::REJECT;
    }
    //Check if the acceleration scaling is a valid value between 0 and 1
    if((goal->acc_scaling <= 0.0) || (goal->acc_scaling > 1.0))
    {
        std::stringstream ss;
        ss << "Goal rejected as the acceleration scaling factor: " << std::to_string(goal->acc_scaling) << " is not possible\n";
        logger_->logError(node_, ss);
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveRobotJointStateActionServer::handleCancel(
    const std::shared_ptr<GoalHandleMoveAction> goal_handle)
{
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveRobotJointStateActionServer::handleAccepted(
    const std::shared_ptr<GoalHandleMoveAction> goal_handle)
{
    std::thread{std::bind(&MoveRobotJointStateActionServer::execute, this, _1), goal_handle}.detach();
}

void MoveRobotJointStateActionServer::execute(const std::shared_ptr<GoalHandleMoveAction> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveAction::Feedback>();
    auto result = std::make_shared<MoveAction::Result>();
    float hertz = 1.0/((float)tree_frequency_/1000);

    std::shared_ptr<bool> success = std::make_shared<bool>(false);
    std::shared_ptr<bool> stop = std::make_shared<bool>(false);

    std::thread{std::bind(&MoveRobotJointStateActionServer::execute_move, this, _1, _2, _3), 
                goal,
                success,
                stop}.detach();

    while((rclcpp::ok()))
    {
        //Check for cancel requests
        if(goal_handle->is_canceling())
        {
            move_group_->stop();
            result->success = false;
            result->message = "Move goal cancelled";
            goal_handle->canceled(result);
            return;
        }
        //Check if the execution has failed
        if(*stop)
        {
            result->success = false;
            result->message = "Move goal failed";
            goal_handle->abort(result);
        }
        goal_handle->publish_feedback(feedback);
        //Check if goal has succeeded
        if(*success){break;} 
        rclcpp::Rate(hertz).sleep();
    }
    result->message = "Trajectory execution complete";
    result->success = true;
    goal_handle->succeed(result);
}

void MoveRobotJointStateActionServer::lifeline()
{
    std_msgs::msg::Int8 msg;
    msg.data = 1;
    action_lifeline_publisher_->publish(msg);
}

void MoveRobotJointStateActionServer::execute_move(const std::shared_ptr<const intervention_msgs::action::MoveRobotJointState_Goal> goal,
                                           const std::shared_ptr<bool> success,
                                           const std::shared_ptr<bool> stop)
{
    //Reset start state
    move_group_->setStartStateToCurrentState();

    //Prep the move_group
    move_group_->setMaxVelocityScalingFactor(goal->vel_scaling);
    move_group_->setMaxAccelerationScalingFactor(goal->acc_scaling);
    move_group_->setJointValueTarget(goal->joint_state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //Plan a path
    //Returns an error if no path is found
    if(!move_group_->plan(plan))
    {
        *stop = true;
        return;
    }
    //Execute the path
    //Returns an error if failed
    if(!move_group_->execute(plan.trajectory_))
    {
        *stop = true;
        return;
    }
    *success = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("move_robot_joint_state_action_server_node");
    MoveRobotJointStateActionServer mrtf(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}