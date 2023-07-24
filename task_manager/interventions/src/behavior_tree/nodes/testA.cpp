#include "interventions/behavior_tree/nodes/testA.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

BT::NodeStatus TestA::onStart()
{
    //As the node needs to get into its RUNNING state ASAP
    //Only the essential STARTUP is done here
    //Long setup is done in initRos()

    //Create empty error message blackboard container
    setOutput("errorMessage", "");
    //Check for initialisation errors
    if(init_error_)
    {
        setOutput("status", "FAILURE");
        setOutput("errorMessage", error_message_);
        return BT::NodeStatus::FAILURE;
    }
    setOutput("status", "STARTING");
    //Get all inputs blackboard
    getInput("sleepTimeMilliseconds", sleepTimeMilliseconds);
    //Standard
    getInput("finalNode", finalNode);
    getInput("prevNodeNumber", nodeNumber);
    getInput("startingNode", startingNode);
    getInput("canInterrupt", canInterrupt);
    {
        //Increase the active node number
        nodeNumber++;
        setOutput("nodeNumber", nodeNumber);
    }
    setOutput("finalNode_out", finalNode);
    setOutput("canInterrupt_out", canInterrupt);

    //Send the action goal
    auto goal_msg = TestAction::Goal();
    goal_msg.sleep_milliseconds = sleepTimeMilliseconds;

    auto send_goal_options = rclcpp_action::Client<TestAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TestA::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TestA::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TestA::resultCallback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);    
    //Create the watchdog for monitoring the connection to the action server
    action_watchdog_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_),
                                                std::bind(&TestA::actionWatchdogCallback, this));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TestA::onRunning()
{
    //If the current nodeNumber is smaller than the starting node number, skip this node
    getInput("startingNode", startingNode);
    if(nodeNumber < startingNode)
    {
        logger_->logWarning(node_, "Node was skipped");
        client_ptr_->async_cancel_all_goals();
        setOutput("status", "SKIPPED");
        current_status_ = BT::NodeStatus::SKIPPED;
        return current_status_;
    }
    if(current_status_ == BT::NodeStatus::FAILURE)
    {
        client_ptr_->async_cancel_all_goals();
        setOutput("status", "FAILURE");
        setOutput("errorMessage", error_message_);
        return current_status_;
    }
    else if(current_status_ == BT::NodeStatus::SUCCESS)
    {
        setOutput("status", "SUCCESS");
        return current_status_;
    }
    //Publish feedback
    std::stringstream ss;
    ss << "Time passed in milliseconds: \n";
    ss << " " + std::to_string(latest_feedback_->passed_milliseconds);
    logger_->logFeedback(node_, ss);
    setOutput("status", "RUNNING");
    return current_status_;
}

void TestA::onHalted()
{
    client_ptr_->async_cancel_all_goals();
    logger_->logWarning(node_, "BT node halted");
}

void TestA::initRos()
{
    action_watchdog_counter_ = 0;
    init_error_ = false;
    node_ = std::make_shared<rclcpp::Node>("testA_bt_node");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    spinThread_ = std::make_shared<std::thread>(std::bind(&TestA::spinNode, this));

    //Parameters
    node_->declare_parameter<int>("tree_frequency", 100);
    node_->declare_parameter<int>("timeout", 0);
    node_->declare_parameter<std::string>("action_name", "");
    node_->declare_parameter<bool>("print", true);
    node_->get_parameter<int>("tree_frequency", tree_frequency_);
    node_->get_parameter<int>("timeout", timeout_);
    node_->get_parameter<std::string>("action_name", action_name_);
    node_->get_parameter<bool>("print", print_);

    logger_ = std::make_shared<Logger>(print_);

    if(action_name_ == "" || action_name_.empty())
    {
        error_message_ = "No valid action name specified";
        logger_->logError(node_, error_message_);
        init_error_ = true;
        return;
    }
    if(tree_frequency_ <= 0)
    {
        error_message_ = "No valid tree update frequency specified";
        logger_->logError(node_, error_message_);
        init_error_ = true;
        return;
    }
    if(timeout_ <= 0)
    {
        error_message_ = "No valid timeout value specified";
        logger_->logError(node_, error_message_);
        init_error_ = true;
        return;
    }
    if((timeout_/tree_frequency_) < 2)
    {
        logger_->logWarning(node_, "Timeout is set smaller than twice the update time! This could lead to severe communication issues!");
    }
    std::stringstream ss;
    ss << "TEST A BT NODE INITIALISATION\n";
    ss << "Action name: " << action_name_ << "\n";
    ss << "Updating frequency: " << std::to_string(tree_frequency_) << " ms \n";
    ss << "Timeout: " << std::to_string(timeout_) << " ms\n";
    logger_->logInfo(node_, ss);

    client_ptr_ = rclcpp_action::create_client<TestAction>(node_, action_name_);
    if(!client_ptr_->wait_for_action_server(3s))
    {
        std::stringstream ss;
        ss << "Action server: " << action_name_ << " not available after waiting \n";
        logger_->logError(node_, ss);
        error_message_ = ss.str();
        init_error_ = true;
        return;
    }   

    //Create subscription to listen to the action server
    action_lifeline_subscriber_ = node_->create_subscription<std_msgs::msg::Int8>("testA_action_lifeline", qos, std::bind(&TestA::actionLifeLineCallback, this, _1));
}

void TestA::goalResponseCallback(const GoalHandleTestAction::SharedPtr &goal_handle)
{
    if(!goal_handle)
    {
        error_message_ = "Goal was rejected by server";
        logger_->logError(node_, error_message_);
        current_status_ = BT::NodeStatus::FAILURE;
    }
    else
    {
        current_status_ = BT::NodeStatus::RUNNING;
    }
}
void TestA::feedbackCallback(GoalHandleTestAction::SharedPtr, const std::shared_ptr<const TestAction::Feedback> feedback)
{
  latest_feedback_ = feedback;
}
void TestA::resultCallback(const GoalHandleTestAction::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        error_message_ = "Goal was aborted";
        logger_->logError(node_, error_message_);
        current_status_ = BT::NodeStatus::FAILURE;
        return;
    case rclcpp_action::ResultCode::CANCELED:
        if(current_status_ == BT::NodeStatus::SKIPPED) {return;}
        logger_->logWarning(node_, "Goal was cancelled");
        current_status_ = BT::NodeStatus::FAILURE;
        return;
    default:
        error_message_ = "Unknown result code";
        logger_->logError(node_, error_message_);
        current_status_ = BT::NodeStatus::FAILURE;
        return;
    }
    if (!result.result->success)
    {
        error_message_ = "Goal failed: " + result.result->message;
        logger_->logError(node_, error_message_);
        current_status_ = BT::NodeStatus::FAILURE;
    }
    else
    {
        current_status_ = BT::NodeStatus::SUCCESS;
    }
}

void TestA::spinNode()
{
    rclcpp::spin(node_);
}
void TestA::actionWatchdogCallback()
{
    if (current_status_ == BT::NodeStatus::RUNNING)
    {
        action_watchdog_counter_++;
        if (action_watchdog_counter_ >= (timeout_/tree_frequency_))
        {
            std::stringstream ss;
            ss << "Lost connection to action server: " << action_name_ << "\n";
            logger_->logError(node_, ss);
            error_message_ = ss.str();
            current_status_ = BT::NodeStatus::FAILURE;
        }
    }
}
void TestA::actionLifeLineCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    //When messages are received, reset the watchdog counter
    if(msg->data == 1)
    {
        action_watchdog_counter_ = 0;
    }
}