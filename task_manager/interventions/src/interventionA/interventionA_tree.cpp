#include "interventions/interventionA/interventionA_tree.hpp"

InterventionATree::InterventionATree(const std::shared_ptr<rclcpp::Node> node)
{
  finished_ = false;
  node_ = node;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  //Create the lifeline publisher outside the namespace of the intervention
  intervention_lifeline_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("/intervention_lifeline", 1);

  node_->declare_parameter<std::string>("action_name", "");
  node_->declare_parameter<int>("tree_frequency", 100);
  node_->declare_parameter<int>("timeout", 0);
  node_->declare_parameter<bool>("print", true);
  node_->get_parameter<std::string>("action_name", action_name_);
  node_->get_parameter<int>("tree_frequency", tree_frequency_);
  node_->get_parameter<int>("timeout", timeout_);
  node_->get_parameter<bool>("print", print_);

  logger_ = std::make_shared<Logger>(print_);

  //Check input parameters
  if(tree_frequency_ <= 0)
  {
    logger_->logError(node_, "No valid tree frequency specified! Exiting");
    return;
  }
  if(action_name_ == "" || action_name_.empty())
  {
    logger_->logError(node_, "No action name specified! Exiting");
    return;
  }
  if(timeout_ <= 0)
  {
      logger_->logError(node_, "No valid timeout value specified! Exiting");
      return;
  }
  if((timeout_/tree_frequency_) < 2)
  {
      logger_->logWarning(node_, "Timeout is set smaller than twice the update time! This could lead to severe communication issues!");
  }
  std::stringstream ss;
  ss << "INTERVENTION A TREE INITIALISATION\n";
  ss << "Action name: " << action_name_ << "\n";
  ss << "Tree update requency: " << std::to_string(tree_frequency_) << " ms \n";
  ss << "Timeout: " << std::to_string(timeout_) << " ms\n";
  logger_->logInfo(node_, ss);


  // Initialise tree
  factory_.registerNodeType<InitMoveRobot>("InitMoveRobot");
  factory_.registerNodeType<FinalMoveRobot>("FinalMoveRobot");
  factory_.registerNodeType<TestA>("TestA");
  factory_.registerNodeType<TestB>("TestB");
  factory_.registerNodeType<MoveRobotTf>("MoveRobotTf");
  factory_.registerNodeType<MoveRobotNamed>("MoveRobotNamed");
  std::string share_path = ament_index_cpp::get_package_share_directory("interventions");
  tree_ = factory_.createTreeFromFile(share_path + "/config/interventionA/interventionA_tree.xml");

  this->action_server_ = rclcpp_action::create_server<ActivateIntervention>(
    node_, 
    action_name_,
    std::bind(&InterventionATree::handleGoal, this, _1, _2),
    std::bind(&InterventionATree::handleCancel, this, _1),
    std::bind(&InterventionATree::handleAccepted, this, _1));
}

rclcpp_action::GoalResponse InterventionATree::handleGoal(const rclcpp_action::GoalUUID &uuid,
                                        std::shared_ptr<const ActivateIntervention::Goal> goal)
{
  logger_->logInfo(node_, "Receiving request to perform intervention A");
  if(goal->start_node != 1)
  {
    std::stringstream ss;
    ss << "Jumping to node: " << std::to_string(goal->start_node) << "\n";
    logger_->logWarning(node_, ss);
  }
  finished_ = false;
  (void) uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse InterventionATree::handleCancel(const std::shared_ptr<GoalHandleActivateIntervention> goal_handle)
{
  logger_->logInfo(node_, "Received cancel request for intervention A");
  (void) goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}                                       

void InterventionATree::handleAccepted(const std::shared_ptr<GoalHandleActivateIntervention> goal_handle)
{
  std::thread{std::bind(&InterventionATree::execute, this, _1), goal_handle}.detach();
  std::thread{std::bind(&InterventionATree::lifeline, this)}.detach();
}

void InterventionATree::execute(const std::shared_ptr<GoalHandleActivateIntervention> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ActivateIntervention::Feedback>();
  auto result = std::make_shared<ActivateIntervention::Result>();
    
  BT::Blackboard::Ptr blackboard = tree_.rootBlackboard();
  //Fill all blackboard keys with starting values
  blackboard->set("status", "");
  blackboard->set("errorMessage", "");
  blackboard->set("nodeNumber", 0);
  blackboard->set("finalNode", false);
  blackboard->set("canInterrupt", true);
  blackboard->set("startingNode", (int) goal->start_node);
  blackboard->set("initJointState", goal->joint_state);
  
  int nodeNumber = 0;
  bool finalNode = false;
  bool canInterrupt = true;
  std::string status_str = "";

  BT::NodeStatus status = tree_.tickOnce();

  while (status == BT::NodeStatus::RUNNING && rclcpp::ok())
  {
    blackboard->get("nodeNumber", nodeNumber);
    blackboard->get("finalNode", finalNode);
    blackboard->get("canInterrupt", canInterrupt);
    blackboard->get("status", status_str);
    std::stringstream ss;
    ss << "Node number: " << std::to_string(nodeNumber) << "\n";
    ss << "Status: " << status_str << "\n";
    logger_->logInfo(node_, ss);
    //Check for cancel requests
    if((goal_handle->is_canceling()) && (status_str != "RUNNING") && (canInterrupt))
    {
        tree_.haltTree();
        result->end_status.status = intervention_msgs::msg::InterventionStatus::CANCELLED;
        goal_handle->canceled(result);
        logger_->logWarning(node_, "Intervention cancelled");
        finished_ = true;
        return;
    }
    // Sleep to avoid busy loops.
    // do NOT use other sleep functions!
    tree_.sleep(std::chrono::milliseconds(tree_frequency_));
    status = tree_.tickOnce();

    feedback->status.status = intervention_msgs::msg::InterventionStatus::RUNNING;
    feedback->intervention = "intervention_A";
    feedback->node_number = nodeNumber;
    feedback->final_node = finalNode;
    goal_handle->publish_feedback(feedback);
  }
  if(status == BT::NodeStatus::FAILURE)
  {
    blackboard->get("errorMessage", error_message_);
    blackboard->get("nodeNumber", nodeNumber);
    std::stringstream ss;
    ss << "Tree result: FAILURE \n";
    ss << "From node: " << std::to_string(nodeNumber) << "\n";
    ss << "Reason: " << error_message_ << "\n";
    logger_->logError(node_, ss);    
    result->end_status.status = intervention_msgs::msg::InterventionStatus::FAILED;
    result->error_message = ss.str();
    finished_ = true;
    goal_handle->succeed(result);
  }
  else if(status == BT::NodeStatus::SUCCESS)
  {
    std::stringstream ss;
    ss << "Tree result: SUCCESS \n";
    logger_->logInfo(node_, ss);   
    result->end_status.status = intervention_msgs::msg::InterventionStatus::SUCCESS;
    finished_ = true;
    goal_handle->succeed(result);
  }
}

void InterventionATree::managerLifelineCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  //If a message is received, reset the watchdog counter
  manager_watchdog_counter_ = 0;
}

void InterventionATree::managerWatchdogCallback()
{
  manager_watchdog_counter_++;
  if (manager_watchdog_counter_ >= (timeout_/tree_frequency_))
  {
      tree_.haltTree();
      logger_->logError(node_, "Lost connection task manager during intervention, HARD SHUTDOWN");
      //Hard shutdown
      rclcpp::shutdown();
  }
}

void InterventionATree::lifeline()
{
  std_msgs::msg::Int8 msg;
  msg.data = 1;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr lifeline_subscriber_ = node_->create_subscription<std_msgs::msg::Int8>("/manager_lifeline", 1, std::bind(&InterventionATree::managerLifelineCallback, this, _1));
  rclcpp::TimerBase::SharedPtr watchdog_timer = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_), std::bind(&InterventionATree::managerWatchdogCallback, this));
  while ((rclcpp::ok()) && (!finished_))
  {
    intervention_lifeline_publisher_->publish(msg);
    rclcpp::Rate(1.0/((float)tree_frequency_/1000)).sleep();
  }
}

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("interventionA_node");
  InterventionATree iat(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
