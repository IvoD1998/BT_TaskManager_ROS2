#pragma once

//ROS components
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//Messages/services/actions
#include "std_msgs/msg/int8.hpp"
#include "intervention_msgs/action/test_action.hpp"
//Logging
#include "interventions/logger.hpp"

class TestBActionServer
{
public:
    using TestAction = intervention_msgs::action::TestAction;
    using GoalHandleTestAction = rclcpp_action::ServerGoalHandle<TestAction>;

    /**
     * @brief Constructor of velocity jog action server
    */
    explicit TestBActionServer(const std::shared_ptr<rclcpp::Node> node);
private:
    /**
     * @brief Callback function for handling goal requests to the action server
    */
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TestAction::Goal> goal);

    /**
     * @brief Callback function for handling cancel requests to the action server
    */
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleTestAction> goal_handle);

    /**
     * @brief Callback function for handling accepted goals
    */
    void handleAccepted(
        const std::shared_ptr<GoalHandleTestAction> goal_handle);

    /**
     * @brief Function for executing accepted goals
    */
    void execute(const std::shared_ptr<GoalHandleTestAction> goal_handle);
    
    /**
     * @brief Function to publish the lifeline of the action server
    */
    void lifeline();

    //Class variables
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Server<TestAction>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr action_lifeline_publisher_;
    rclcpp::TimerBase::SharedPtr action_lifeline_timer_;
    std::shared_ptr<Logger> logger_;
    
    // Node parameters
    int tree_frequency_;
    std::string action_name_;
    bool print_;
};