#include "rclcpp/rclcpp.hpp"
#include "interventions/logger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "intervention_msgs/action/activate_intervention.hpp"
#include "intervention_msgs/srv/trigger_intervention.hpp"
#include "intervention_msgs/srv/change_priority.hpp"
#include "intervention_msgs/srv/completion_confirmation.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "intervention_msgs/msg/intervention_status.hpp"
#include "std_msgs/msg/int8.hpp"
#include "interventions/intervention_object.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * @brief TaskManager management and planning class of the intervention system
*/
class TaskManager
{
public:
    using ActivateIntervention = intervention_msgs::action::ActivateIntervention;
    using GoalHandleActivateIntervention = rclcpp_action::ClientGoalHandle<ActivateIntervention>;

    /**
     * @brief Constructor of the class
     * @param node is the ROS node, used for handling all ROS-related components
    */
    TaskManager(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Destructor of the class
    */
    ~TaskManager();

private:

    /**
     * @brief Callback function which is triggered every time a goal request has been accepted/denied
     * @param goal_handle is the goal handle of the requested intervention
    */
    void GoalResponseCallback(const GoalHandleActivateIntervention::SharedPtr &goal_handle);

    /**
     * @brief Callback function which is triggered every time the action server gives feedback of the action
     * @param feedback is the feedback message containing the feedback
    */
    void feedbackCallback(GoalHandleActivateIntervention::SharedPtr, const std::shared_ptr<const ActivateIntervention::Feedback> feedback);

    /**
     * @brief Callback function which is triggered every time an action returns a result
     * @param result is the result message containing the result information
    */
    void resultCallback(const GoalHandleActivateIntervention::WrappedResult &result);

    /**
     * @brief Callback function which is triggered every time a new intervention execution is requested. The interventions are placed inside the intervention buffer
     * @param req is the requested intervention
     * @param res is the response message sent back to the service client
    */
    void interventionTriggerCallback(const intervention_msgs::srv::TriggerIntervention::Request::SharedPtr req,
                                intervention_msgs::srv::TriggerIntervention::Response::SharedPtr res);

    /**
     * @brief Callback function which is triggered by the monitor when the priority of a specific intervention should be changed
     * @param req is the change request containing the new priority level and the UUID of the corresponding intervention
     * @param res is the response message sent back to the service client
    */
    void changePriorityCallback(const intervention_msgs::srv::ChangePriority::Request::SharedPtr req,
                                intervention_msgs::srv::ChangePriority::Response::SharedPtr res);

    /**
     * @brief Callback function which is triggered at a given rate, filtering the intervention buffer to the right order of intervention queue
    */
    void bufferTimerCallback();

    /**
     * @brief Callback function which is triggered at a given rate, handling the intervention queue and managing the active intervention
    */
    void queueTimerCallback();

    /**
     * @brief Callback function which monitors the connection to the action server. If connection is lost, the watchdog will throw an error
    */
    void interventionWatchdogCallback();

    /**
     * @brief Callback function receiving data from the interventions as they enter their running state, used for monitoring connection
    */
    void interventionLifelineCallback(const std_msgs::msg::Int8::SharedPtr msg);

    /**
     * @brief Callback function which publishes an integer value at the given tree_frequency as a heartbeat for monitoring by other executables
    */
    void managerLifelineCallback();

    /**
     * @brief Callback function which reads the latest joint state of the robot
    */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    //Private variables
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::TimerBase::SharedPtr intervention_watchdog_timer_;
    int intervention_watchdog_counter_;
    std::shared_ptr<const ActivateIntervention::Feedback> latest_feedback_;
    rclcpp::Service<intervention_msgs::srv::TriggerIntervention>::SharedPtr intervention_trigger_;
    rclcpp::Service<intervention_msgs::srv::ChangePriority>::SharedPtr change_priority_server_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr manager_lifeline_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr intervention_lifeline_subscriber_;
    rclcpp::TimerBase::SharedPtr buffer_timer_, queue_timer_, manager_lifeline_timer_;
    std::vector<std::shared_ptr<Intervention>> intervention_buffer_, intervention_queue_;
    std::shared_ptr<Intervention> current_intervention_;
    int tree_frequency_, timeout_;
    sensor_msgs::msg::JointState latest_joint_state_;

    //Logging
    std::shared_ptr<Logger> logger_;
    std::string checkString_;
    bool print_;

    //time debug
    rclcpp::Time queueTime_, bufferTime_, feedbackTime_;
};
