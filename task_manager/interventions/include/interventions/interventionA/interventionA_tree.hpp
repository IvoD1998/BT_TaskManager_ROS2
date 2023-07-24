#pragma once

//ROS components
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
//Messages/services/actions
#include "intervention_msgs/action/activate_intervention.hpp"
#include "intervention_msgs/msg/intervention_status.hpp"
#include "std_msgs/msg/int8.hpp"
//Behavior tree
#include "behaviortree_cpp/bt_factory.h"
#include "interventions/behavior_tree/nodes/init_nodes/init_move.hpp"
#include "interventions/behavior_tree/nodes/final_nodes/final_move.hpp"
#include "interventions/behavior_tree/nodes/testA.hpp"
#include "interventions/behavior_tree/nodes/testB.hpp"
#include "interventions/behavior_tree/nodes/move_robot_tf.hpp"
#include "interventions/behavior_tree/nodes/move_robot_named.hpp"
//Logging
#include "interventions/logger.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class InterventionATree
{
public:
    using ActivateIntervention = intervention_msgs::action::ActivateIntervention;
    using GoalHandleActivateIntervention = rclcpp_action::ServerGoalHandle<ActivateIntervention>;

    /**
     * @brief Constructor for the Intervention A behavior tree class
     * @param node is the ROS node for constructing all ROS-related components
    */
    explicit InterventionATree(const std::shared_ptr<rclcpp::Node> node);

private:
    /**
     * @brief Callback function when a goal request is received for executing the behavior tree
     * @param uuid is the unique identifier of the goal request
     * @param goal is the goal request message
    */
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const ActivateIntervention::Goal> goal);

    /**
     * @brief Callback function when a cancel request is received
     * @param goal_handle is the goal handle of the goal which should be cancelled
    */
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleActivateIntervention> goal_handle);

    /**
     * @brief Callback function when a goal is accepted and should be executed
     * @param goal_handle is the goal handle of the corresponding goal request
    */
    void handleAccepted(const std::shared_ptr<GoalHandleActivateIntervention> goal_handle);

    /**
     * @brief function which runs the execution of the behavior tree after acceptation
    */
    void execute(const std::shared_ptr<GoalHandleActivateIntervention> goal_handle);

    /**
     * @brief Callback function, listening to the lifeline of the manager and resetting its watchdog counter
    */
    void managerLifelineCallback(const std_msgs::msg::Int8::SharedPtr msg);

    /**
     * @brief Callback function guarding the connection to the manager
    */
    void managerWatchdogCallback();

    /**
     * @brief Function to publish the lifeline of the intervention, and to connect to the managers lifeline
    */
    void lifeline();

    //Class variables
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Server<ActivateIntervention>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr intervention_lifeline_publisher_;
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    int manager_watchdog_counter_;
    bool finished_;

    //Logging
    std::shared_ptr<Logger> logger_;
    std::string error_message_;

    // Node parameters
    int tree_frequency_, timeout_;
    std::string action_name_;
    bool print_;
};
