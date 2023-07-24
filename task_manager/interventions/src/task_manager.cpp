#include "interventions/task_manager.hpp"

TaskManager::TaskManager(std::shared_ptr<rclcpp::Node> node)
{
    checkString_ = "";
    node_ = node;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    current_intervention_ = std::make_shared<Intervention>();
    intervention_queue_ = {};
    intervention_buffer_ = {};
    //TODO update parameters to configuration file
    tree_frequency_ = 100;
    timeout_ = 500;
    print_ = true;

    logger_ = std::make_shared<Logger>(print_, true);

    manager_lifeline_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("manager_lifeline", 1);
    intervention_lifeline_subscriber_ = node_->create_subscription<std_msgs::msg::Int8>("/intervention_lifeline", qos, std::bind(&TaskManager::interventionLifelineCallback, this, _1));
    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", qos, std::bind(&TaskManager::jointStateCallback, this, _1));
    manager_lifeline_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_), std::bind(&TaskManager::managerLifelineCallback, this));
    intervention_trigger_ = node_->create_service<intervention_msgs::srv::TriggerIntervention>("trigger_intervention", std::bind(&TaskManager::interventionTriggerCallback, this, _1, _2));
    change_priority_server_ = node_->create_service<intervention_msgs::srv::ChangePriority>("change_priority", std::bind(&TaskManager::changePriorityCallback, this, _1, _2));
    buffer_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_/10), std::bind(&TaskManager::bufferTimerCallback, this));
    queue_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_), std::bind(&TaskManager::queueTimerCallback, this));
}   

TaskManager::~TaskManager()
{
    if((current_intervention_->name != "") || (!current_intervention_->name.empty()))
        current_intervention_->action_client->async_cancel_all_goals();
}

void TaskManager::GoalResponseCallback(const GoalHandleActivateIntervention::SharedPtr &goal_handle)
{
    current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::STARTING;
    current_intervention_->goal_handle = goal_handle;
    if(!goal_handle)
    {
        logger_->logError(node_, "Goal was rejected by server");
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::REJECTED;
        return;
    }
    else
    {
        logger_->logInfo(node_, "Goal was accepted by server, waiting for result");
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::ACCEPTED;
        // watchdog_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_),
        //                                             std::bind(&TaskManager::watchdogCallback, this));
    }
}
    
void TaskManager::feedbackCallback(GoalHandleActivateIntervention::SharedPtr, 
                            const std::shared_ptr<const ActivateIntervention::Feedback> feedback)
{
    rclcpp::Time stamp = node_->now();
    latest_feedback_ = feedback;

    switch (feedback->status.status)
    {
    case intervention_msgs::msg::InterventionStatus::RUNNING:
        current_intervention_->nodeNumber = feedback->node_number;
        current_intervention_->finalNode = feedback->final_node;
        break;
    
    default:
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::FAILED;
        break;
    }

    std::string str = "feedback loop: " + std::to_string(1/(stamp.seconds() - feedbackTime_.seconds())) + "\n";
    feedbackTime_ = stamp;
    // logger_->logFeedback(node_, ss);
}

void TaskManager::resultCallback(const GoalHandleActivateIntervention::WrappedResult &result)
{
    std::stringstream ss;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        // ss << "Active intervention: " << current_intervention_->name << " ABORTED\n";
        // logger_->logError(node_, ss, checkString_);
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::FAILED;
        current_intervention_->error_message = result.result->error_message;
        return;
    case rclcpp_action::ResultCode::CANCELED:
        // ss << "Active intervention: " << current_intervention_->name << " CANCELLED\n";
        // logger_->logWarning(node_, ss, checkString_);
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::CANCELLED;
        return;
    default:
        logger_->logError(node_, "Unknown result code", checkString_);
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::FAILED;
        current_intervention_->error_message = "Unknown result code";
        return;
    }
    if((result.result->end_status.status) != result.result->end_status.SUCCESS)
    {
        // ss << "Active intervention: " << current_intervention_->name << " FAILED\n";
        // logger_->logError(node_, ss, checkString_);
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::FAILED;
        current_intervention_->error_message = result.result->error_message;
        return;
    }
    else
    {
        // ss << "Active intervention: " << current_intervention_->name << " SUCCEEDED\n";
        // logger_->logInfo(node_, ss, checkString_);
        current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::SUCCESS;
        return ;
    }
}

void TaskManager::interventionTriggerCallback(const intervention_msgs::srv::TriggerIntervention::Request::SharedPtr req,
                                       intervention_msgs::srv::TriggerIntervention::Response::SharedPtr res)
{
    std::string action_name, confirmation_completion_server;
    if(req->intervention == "intervention_A")
    {
        action_name = "/interventionA/activate";
        // confirmation_completion_server = "/interventionA/confirmation_completion";
        confirmation_completion_server = "/confirmation_completion";
    }
    else if(req->intervention == "intervention_B")
    {
        action_name = "/interventionB/activate";
        // confirmation_completion_server = "/interventionB/confirmation_completion";
        confirmation_completion_server = "/confirmation_completion";
    }
    else
    {
        logger_->logError(node_, "Specified intervention name is not recognised!");
        res->message = "Specified intervention name is not recognised!";
        res->success = false;
        return;
    }
    rclcpp_action::Client<ActivateIntervention>::SharedPtr client_ptr;
    client_ptr = rclcpp_action::create_client<ActivateIntervention>(node_, action_name);
    if(!client_ptr->wait_for_action_server())
    {
        std::stringstream ss;
        ss << "Action server: '" + action_name + "' not available after waiting\n";
        logger_->logError(node_, ss);
        res->message = ss.str();
        res->success = false;
        return;
    }
    auto goal_msg = ActivateIntervention::Goal();

    auto send_goal_options = rclcpp_action::Client<ActivateIntervention>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TaskManager::GoalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TaskManager::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TaskManager::resultCallback, this, _1);

    //Create the map of hte intervention and a UUID
    std::shared_ptr<Intervention> intervention = std::make_shared<Intervention>();
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::stringstream temp;
    temp << uuid;
    intervention->uuid = temp.str();
    intervention->name = req->intervention;
    intervention->priority = req->priority;
    intervention->status.status = intervention_msgs::msg::InterventionStatus::QUEUED;
    intervention->action_client = client_ptr;
    intervention->goal_msg = goal_msg;
    intervention->send_goal_options = send_goal_options;
    intervention->completion_confirmation_client = node_->create_client<intervention_msgs::srv::CompletionConfirmation>(confirmation_completion_server);

    //Add to buffer
    intervention_buffer_.insert(intervention_buffer_.begin(), intervention);

    // client_ptr_->async_send_goal(goal_msg, send_goal_options);
    // intervention_queue_.push_back(intervention);
    std::stringstream ss;
    ss << "Intervention: '" << req->intervention << "' \n";
    ss << "With priority: " << std::to_string(req->priority) << "\n";  
    ss << "added to buffer\n";
    logger_->logInfo(node_, ss);
    res->message = "Service request processed successfully";
    res->success = true;
    res->uuid = intervention->uuid;
}

void TaskManager::changePriorityCallback(const intervention_msgs::srv::ChangePriority::Request::SharedPtr req,
                                  intervention_msgs::srv::ChangePriority::Response::SharedPtr res)
{
    //lock memory
    // std::lock_guard<std::mutex> guard(m);
    std::vector<std::shared_ptr<Intervention>>::iterator it;
    //First, search in the queue for the corresponding UUID
    //Use a predicate to find the UUID value
    std::string id = req->uuid;
    //If the uuid belongs to the current intervention
    if(current_intervention_->uuid == req->uuid)
    {
        current_intervention_->priority = req->new_priority;
        std::stringstream ss;
        ss << "Intervention: '" << current_intervention_->name << "'\n";
        ss << "With UUID: " << current_intervention_->uuid << "\n";
        ss << "Found as active intervention\n";
        ss << "Priority changed to: " << std::to_string(req->new_priority) << "\n";
        logger_->logWarning(node_, ss);

        res->success = true;
        res->message = "Intervention found";
        //Check if the first intervention in the queue has a higher priority now
        //If so, add it back to the buffer
        if((!intervention_queue_.empty()) && (intervention_queue_[0]->priority > current_intervention_->priority))
        {
            //Cancel intervention
            current_intervention_->action_client->async_cancel_goal(current_intervention_->goal_handle);
        }
        return;
    }

    auto pred = [id](const std::shared_ptr<Intervention> &intervention){
        return intervention->uuid == id;
    };
    it = std::find_if(intervention_queue_.begin(), intervention_queue_.end(), pred);
    if(it!=intervention_queue_.end())
    {
        //Change priority value
        intervention_queue_[it - intervention_queue_.begin()]->priority = req->new_priority;
        //Move back to buffer for resorting
        intervention_buffer_.push_back(intervention_queue_[it - intervention_queue_.begin()]);
        intervention_queue_.erase(it);

        std::stringstream ss;
        ss << "Intervention: '" << intervention_queue_[it - intervention_queue_.begin()]->name << "'\n";
        ss << "With UUID: " << intervention_queue_[it - intervention_queue_.begin()]->uuid << "\n";
        ss << "Found in queue\n";
        ss << "Priority changed to: " << std::to_string(req->new_priority) << "\n";
        logger_->logWarning(node_, ss);

        res->success = true;
        res->message = "Intervention found";
        return;
    }
    //If its not found in the queue, look in the buffer
    it = std::find_if(intervention_buffer_.begin(), intervention_buffer_.end(), pred);
    if(it!=intervention_buffer_.end())
    {
        intervention_buffer_[it - intervention_buffer_.begin()]->priority = req->new_priority;
        std::stringstream ss;
        ss << "Intervention: '" << intervention_queue_[it - intervention_queue_.begin()]->name << "'\n";
        ss << "With UUID: " << intervention_queue_[it - intervention_queue_.begin()]->uuid << "\n";
        ss << "Found in buffer\n";
        ss << "Priority changed to: " << std::to_string(req->new_priority) << "\n";
        logger_->logWarning(node_, ss);

        res->success = true;
        res->message = "Intervention found";
        return;
    }
    std::stringstream ss;
    ss << "Intervention with UUID: '" << req->uuid << "' not found!\n";
    logger_->logError(node_, ss); 
    res->success = false;
    res->message = "Intervention not found";
    return;
}


void TaskManager::bufferTimerCallback()
{
    //If the buffer is empty, do nothing
    if(intervention_buffer_.empty())
    {
        return;
    }
    //Check if the priority is higher than the current intervention
    if((intervention_buffer_[0]->priority > current_intervention_->priority))
    {
        //Cancel the current intervention (if there is one) and put the new intervention in the front of the buffer
        if((current_intervention_->name != "") || (!current_intervention_->name.empty()))
        {
            logger_->logWarning(node_, "Intervention with higher priority found, interrupting current intervention");
            current_intervention_->action_client->async_cancel_goal(current_intervention_->goal_handle);
        }
    }

    //If the queue is empty, simply put the first element of the buffer in the queue
    if(intervention_queue_.empty())
    {
        intervention_queue_.push_back(intervention_buffer_[0]);
        intervention_buffer_.erase(intervention_buffer_.begin());
        return;
    }
    //Check for priority order
    for(size_t j = 0; j < intervention_queue_.size(); j++)
    {
        //If the buffer entry has a higher priority than the current intervention queue entry, insert it before it
        if(intervention_buffer_[0]->priority > intervention_queue_[j]->priority)
        {
            //Insert the intervention in the first spot of its priority
            intervention_queue_.insert(intervention_queue_.begin() + j, intervention_buffer_[0]);
            //Erase the intervention from the buffer
            intervention_buffer_.erase(intervention_buffer_.begin());
            return;
        }
        //If the buffer entry has a higher node number (unfinished intervention)
        else if((intervention_buffer_[0]->priority == intervention_queue_[j]->priority) &&
                (intervention_buffer_[0]->nodeNumber > intervention_queue_[j]->nodeNumber))
        {
            //Insert the intervention before other interventions of its priority with a lower node number
            intervention_queue_.insert(intervention_queue_.begin() + j, intervention_buffer_[0]);
            //Erase the intervention from the buffer
            intervention_buffer_.erase(intervention_buffer_.begin());
            return;
        }
    }
    intervention_queue_.push_back(intervention_buffer_[0]);
    intervention_buffer_.erase(intervention_buffer_.begin());
}

void TaskManager::queueTimerCallback()
{
    rclcpp::Time stamp = node_->now();
    //If nothing to do
    if(intervention_queue_.empty() && (current_intervention_->name == "" || current_intervention_->name.empty()))
    {
        logger_->logInfo(node_, "TaskManager node in IDLE state...", checkString_);
        return;
    }
    else
    {
        if(current_intervention_->name == "" || current_intervention_->name.empty())
        {   
            //Set first intervention in queue as the active one
            current_intervention_ = intervention_queue_[0];
            //Delete the intervention from the queue
            if(current_intervention_->nodeNumber == 0)
            {
                std::stringstream ss;
                ss << "Setting intervention: '" << current_intervention_->name << "' as current intervention and removing it from the queue\n";
                ss << "With priority: " << std::to_string(current_intervention_->priority) << "\n";
                ss << "UUID: " << current_intervention_->uuid << "\n";
                //Set the starting joint state of the intervention to the current joint state of the robot
                current_intervention_->joint_state = latest_joint_state_;
                logger_->logInfo(node_, ss);
            }
            else
            {
                std::stringstream ss;
                ss << "Revisiting intervention: '" << current_intervention_->name << "' as current intervention and removing it from the queue\n";
                ss << "With priority: " << std::to_string(current_intervention_->priority) << "\n";
                ss << "UUID: " << current_intervention_->uuid << "\n";
                ss << "Starting at node: " << std::to_string(current_intervention_->nodeNumber + 1) << "\n";
                logger_->logWarning(node_, ss);
            }
            intervention_queue_.erase(intervention_queue_.begin());
            //Print the queue
            std::stringstream ss;
            ss << "Current queue:\n";
            for(size_t i = 0; i < intervention_queue_.size(); i++)
            {
                ss << std::to_string(i) << ")";
                ss << " Name: " << intervention_queue_[i]->name;
                ss << "; Priority: " << std::to_string(intervention_queue_[i]->priority);
                ss << "; Starting node: " << std::to_string(intervention_queue_[i]->nodeNumber + 1); 
                ss << "' UUID: " << intervention_queue_[i]->uuid << "\n";
            }
            logger_->logInfo(node_, ss, checkString_);
            current_intervention_->goal_msg.start_node = current_intervention_->nodeNumber + 1;
            current_intervention_->goal_msg.joint_state = current_intervention_->joint_state;
            current_intervention_->action_client->async_send_goal(current_intervention_->goal_msg, current_intervention_->send_goal_options);
        }
        else
        {
            std::stringstream ss;
            switch (current_intervention_->status.status)
            {
            case intervention_msgs::msg::InterventionStatus::QUEUED:
                ss << "Intervention: '" << current_intervention_->name << "' in QUEUED phase\n";
                logger_->logInfo(node_, ss, checkString_);
                break;
            case intervention_msgs::msg::InterventionStatus::ACCEPTED:
                ss << "Intervention: '" << current_intervention_->name << "' ACCEPTED by server\n";
                logger_->logInfo(node_, ss, checkString_);
                current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::RUNNING;
                intervention_watchdog_timer_ = node_->create_wall_timer(std::chrono::milliseconds(tree_frequency_),
                                                    std::bind(&TaskManager::interventionWatchdogCallback, this));
                break;

            case intervention_msgs::msg::InterventionStatus::STARTING:
                ss << "Intervention: '" << current_intervention_->name << "' in STARTING phase\n";
                logger_->logInfo(node_, ss, checkString_);
                break;
            case intervention_msgs::msg::InterventionStatus::RUNNING:
                ss << "Intervention: '" << current_intervention_->name << "' in RUNNING phase\n";
                ss << "Priority: " << std::to_string(current_intervention_->priority) << "\n";
                ss << "Node number " << std::to_string(current_intervention_->nodeNumber) << "\n";
                logger_->logInfo(node_, ss, checkString_);
                break;
            case intervention_msgs::msg::InterventionStatus::SUCCESS:
            {
                ss << "Intervention: '" << current_intervention_->name << "' SUCCEEDED\n";
                logger_->logInfo(node_, ss, checkString_);
                //Empty out the current_intervention_ container
                //TODO: put successful interventions in SUCCESS container
                auto request = std::make_shared<intervention_msgs::srv::CompletionConfirmation::Request>();
                request->uuid = current_intervention_->uuid;
                auto result = current_intervention_->completion_confirmation_client->async_send_request(request);
                current_intervention_ = std::make_shared<Intervention>();
                break;
            }
            case intervention_msgs::msg::InterventionStatus::FAILED:
            {
                ss << "Intervention: '" << current_intervention_->name << "' FAILED\n";
                ss << current_intervention_->error_message << "\n";
                logger_->logError(node_, ss, checkString_);
                //Empty out the current_intervention_ container
                //TODO: Put failed interventions in FAILED container
                auto request = std::make_shared<intervention_msgs::srv::CompletionConfirmation::Request>();
                request->uuid = current_intervention_->uuid;
                auto result = current_intervention_->completion_confirmation_client->async_send_request(request);
                current_intervention_ = std::make_shared<Intervention>();
                break;
            }
            case intervention_msgs::msg::InterventionStatus::CANCELLED:
                ss << "Intervention: '" << current_intervention_->name << "' CANCELLED\n";
                //Add the intervention back to the buffer
                if(!current_intervention_->finalNode)
                {   
                    //The intervention is not finished yet, so put it back in the buffer
                    ss << "intervention not finished, adding back to queue\n";                  
                    logger_->logWarning(node_, ss, checkString_);
                    //Save the current joint state
                    current_intervention_->joint_state = latest_joint_state_; 
                    intervention_buffer_.insert(intervention_buffer_.begin(), current_intervention_);
                }
                //Empty out the current_intervention_ container
                current_intervention_ = std::make_shared<Intervention>();
                break;
            case intervention_msgs::msg::InterventionStatus::REJECTED:
            {
                ss << "Intervention: '" << current_intervention_->name << "' REJECTED\n";
                logger_->logWarning(node_, ss, checkString_);
                //Empty out the current_intervention_ container
                auto request = std::make_shared<intervention_msgs::srv::CompletionConfirmation::Request>();
                request->uuid = current_intervention_->uuid;
                auto result = current_intervention_->completion_confirmation_client->async_send_request(request);
                current_intervention_ = std::make_shared<Intervention>();
                break;
            }
            default:
                ss << "Unknown status code\n";
                logger_->logError(node_, ss, checkString_);
                break;
            }
        }
    }
    std::string str = "queue loop: " + std::to_string(1/(stamp.seconds() - queueTime_.seconds())) + "\n";
    // logger_->logInfo(node_, str);
    queueTime_ = stamp;
}

void TaskManager::interventionWatchdogCallback()
{
    switch (current_intervention_->status.status)
    {
    case intervention_msgs::msg::InterventionStatus::RUNNING:
        intervention_watchdog_counter_++;
        if (intervention_watchdog_counter_ >= (timeout_/tree_frequency_))
        {
            std::stringstream ss;
            logger_->logError(node_, "Lost connection to intervention tree");
            current_intervention_->status.status = intervention_msgs::msg::InterventionStatus::FAILED;
            rclcpp::shutdown();
        }
        break;
    
    default:
        break;
    }
}

void TaskManager::interventionLifelineCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    if(msg->data == 1)
    {
        intervention_watchdog_counter_ = 0;
    }
}

void TaskManager::managerLifelineCallback()
{
    std_msgs::msg::Int8 msg;
    msg.data = 1;
    manager_lifeline_publisher_->publish(msg);
}

void TaskManager::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    latest_joint_state_ = *msg;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("task_manager_node");
    TaskManager m(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}