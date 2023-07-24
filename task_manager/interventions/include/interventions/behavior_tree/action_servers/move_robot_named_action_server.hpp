#pragma once

//ROS components
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//Messages/services/actions
#include "std_msgs/msg/int8.hpp"
#include "intervention_msgs/action/move_robot_named.hpp"
//Logging
#include "interventions/logger.hpp"
//Moveit
#include "moveit/move_group_interface/move_group_interface.h"

class MoveRobotNamedActionServer
{
public:
    using MoveAction = intervention_msgs::action::MoveRobotNamed;
    using GoalHandleMoveAction = rclcpp_action::ServerGoalHandle<MoveAction>;

    /**
     * @brief Constructor of the action server
     * @param node ROS node for controlling ROS-related components
    */
    explicit MoveRobotNamedActionServer(const std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Destructor of the action server
    */
    ~MoveRobotNamedActionServer()
    {
        move_group_->stop();
    }
private:
    /**
     * @brief Callback function for handling goal requests to the action server
    */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveAction::Goal> goal);

    /**
     * @brief Callback function for handling cancel requests to the action server
    */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleMoveAction> goal_handle);

    /**
     * @brief Callback function for handling accepted goals
    */
    void handleAccepted(
        const std::shared_ptr<GoalHandleMoveAction> goal_handle);

    /**
     * @brief Function for executing accepted goals
    */
    void execute(const std::shared_ptr<GoalHandleMoveAction> goal_handle);

    /**
     * @brief Function to publish the lifeline of the action server
    */
    void lifeline();

    void execute_move(const std::shared_ptr<const intervention_msgs::action::MoveRobotNamed_Goal> goal,
                      const std::shared_ptr<bool> success,
                      const std::shared_ptr<bool> stop);

    //Class variables
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Server<MoveAction>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr action_lifeline_publisher_;
    rclcpp::TimerBase::SharedPtr action_lifeline_timer_;
    std::shared_ptr<Logger> logger_;

    //Additional
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    //Node parameters
    int tree_frequency_;
    std::string action_name_, robot_name_;
    bool print_;
};
