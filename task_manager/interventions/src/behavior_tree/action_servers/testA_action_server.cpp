#include "interventions/behavior_tree/action_servers/testA_action_server.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

TestAActionServer::TestAActionServer(const std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    
    //Lifelines for actions are specified per server and inside the namespace to prevent clashing
    //and correct function when the same action server is used in multiple interventions
    action_lifeline_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("testA_action_lifeline", 1);

    //Parameters
    node_->declare_parameter<int>("tree_frequency", 100);
    node_->declare_parameter<std::string>("action_name", "");
    node_->declare_parameter<bool>("print", true);
    node_->get_parameter<int>("tree_frequency", tree_frequency_);
    node_->get_parameter<std::string>("action_name", action_name_);
    node_->get_parameter<bool>("print", print_);

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

    //Print initialisation message
    std::stringstream ss;
    ss << "TEST A ACTION SERVER INITIALISATION\n";
    ss << "Action name: " << action_name_ << "\n";
    ss << "Tree update frequency: " << std::to_string(tree_frequency_) << " ms\n";
    logger_->logInfo(node_, ss);

    //Create the lifeline publisher timer
    action_lifeline_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_), std::bind(&TestAActionServer::lifeline, this));

    //Create action server
    this->action_server_ = rclcpp_action::create_server<TestAction>(
        node_,
        action_name_,
        std::bind(&TestAActionServer::handleGoal, this, _1, _2),
        std::bind(&TestAActionServer::handleCancel, this, _1),
        std::bind(&TestAActionServer::handleAccepted, this, _1));
}

rclcpp_action::GoalResponse TestAActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const TestAction::Goal> goal)
{
    //Check if the input is valid
    if(goal->sleep_milliseconds <= 0)
    {
        logger_->logError(node_, "Can't sleep as the sleep time given is not a positive integer");
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void) uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
    
rclcpp_action::CancelResponse TestAActionServer::handleCancel(
    const std::shared_ptr<GoalHandleTestAction> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TestAActionServer::handleAccepted(
    const std::shared_ptr<GoalHandleTestAction> goal_handle)
{
    std::thread{std::bind(&TestAActionServer::execute, this, _1), goal_handle}.detach();
}

void TestAActionServer::execute(const std::shared_ptr<GoalHandleTestAction> goal_handle)
{        
    //The node should start sending feedback as soon as possible
    //Prevent long blocking operations before the feedback loop
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TestAction::Feedback>();
    auto result = std::make_shared<TestAction::Result>();
    float hertz = 1.0/((float)tree_frequency_/1000);
    
    int sleepTime = 0;
    while (rclcpp::ok())
    {
        //Check for cancel request
        if(goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Sleeping cancelled";
            goal_handle->canceled(result);
            return;
        }
        if(sleepTime >= goal->sleep_milliseconds)
        {
            break;
        }
        sleepTime += tree_frequency_;
        feedback->passed_milliseconds = sleepTime;
        goal_handle->publish_feedback(feedback);
        rclcpp::Rate(hertz).sleep();
    }
    
    result->message = "Sleeping completed";
    result->success = true;
    goal_handle->succeed(result);
}

void TestAActionServer::lifeline()
{
    std_msgs::msg::Int8 msg;
    msg.data = 1;
    action_lifeline_publisher_->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("testA_action_server_node");
    TestAActionServer iaac(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}