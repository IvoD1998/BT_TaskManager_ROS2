#pragma once

//ROS components
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//Messages/services/actions
#include "std_msgs/msg/int8.hpp"
#include "intervention_msgs/action/move_robot_named.hpp"
//Logging
#include "interventions/logger.hpp"
//Behavior tree
#include "behaviortree_cpp/action_node.h"

class MoveRobotNamed : public BT::StatefulActionNode
{
public: 
    using MoveAction = intervention_msgs::action::MoveRobotNamed;
    using GoalHandleMoveAction = rclcpp_action::ClientGoalHandle<MoveAction>;

    /**
     * @brief Constructor for behavior tree node 
    */
    MoveRobotNamed(const std::string &name, const BT::NodeConfig &config) : StatefulActionNode(name, config)
    {
        initRos();
    } 

    /**
     * @brief Destructor of class
    */
    ~MoveRobotNamed()
    {
        spinThread_->detach();
    }

    /**
     * @brief List of all input and output ports available for the behavior tree node
    */   
    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<std::string>("namedGoal"),
               BT::InputPort<float>("velocityScaling"),
               BT::InputPort<float>("accelerationScaling"),
               BT::InputPort<int>("prevNodeNumber"),
               BT::InputPort<bool>("finalNode"),
               BT::InputPort<int>("startingNode"),
               BT::InputPort<bool>("canInterrupt"),
               BT::OutputPort<int>("nodeNumber"),
               BT::OutputPort<bool>("finalNode_out"),
               BT::OutputPort<bool>("canInterrupt_out"),
               BT::OutputPort<std::string>("status"),
               BT::OutputPort<std::string>("errorMessage")};
    }

    /**
     * @brief Function executed at the start of the behavior tree node
    */
    BT::NodeStatus onStart() override;

    /**
     * @brief Function executed in the running state of the behavior tree node
    */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Function executed when behavior tree node is halted
    */
    void onHalted() override;

private:

    /**
     * @brief Initialisation of class object
    */
    void initRos();

    /**
     * @brief Function executed when receiving a response from the action server regarding the goal request acceptance
     * @param goal_handle the goal handle of the goal request
    */
    void goalResponseCallback(const GoalHandleMoveAction::SharedPtr &goal_handle);

    /**
     * @brief Function executed when receiving feedback from the action server
     * @param feedback the feedback message
    */
    void feedbackCallback(GoalHandleMoveAction::SharedPtr, const std::shared_ptr<const MoveAction::Feedback> feedback);

    /**
     * @brief Function executed when receiving the result of the action after completion
     * @param result the result message
    */
    void resultCallback(const GoalHandleMoveAction::WrappedResult &result);

    /**
     * @brief Function to spin the ROS node on a seperate thread
    */
    void spinNode();

    /**
     * @brief Function monitoring the connection to the action server it throws an error if it is broken
    */
    void actionWatchdogCallback();

    /**
     * @brief Function to listen to the lifeline of the action server and reset the watchdog
    */
    void actionLifeLineCallback(const std_msgs::msg::Int8::SharedPtr msg);

    //Class variables
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<std::thread> spinThread_;
    rclcpp_action::Client<MoveAction>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr action_lifeline_subscriber_;
    rclcpp::TimerBase::SharedPtr action_watchdog_timer_;
    std::shared_ptr<const MoveAction::Feedback> latest_feedback_;
    int action_watchdog_counter_;
    BT::NodeStatus current_status_;
    bool init_error_;
    std::shared_ptr<Logger> logger_;
    std::string error_message_;

    //Node parameters
    int tree_frequency_, timeout_;
    std::string action_name_;
    bool print_;

    //Blackboard variables
    std::string namedGoal;
    float velocityScaling, accelerationScaling;
    int  nodeNumber, startingNode;
    bool finalNode, canInterrupt;

};