#include "rclcpp_action/rclcpp_action.hpp"
#include "intervention_msgs/action/activate_intervention.hpp"
#include "intervention_msgs/msg/intervention_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "intervention_msgs/srv/completion_confirmation.hpp"


/**
 * @brief Struct for the 'intervention' object container
*/
struct Intervention
{
    std::string name = "";
    intervention_msgs::msg::InterventionStatus status;
    rclcpp_action::Client<intervention_msgs::action::ActivateIntervention>::SharedPtr action_client;
    intervention_msgs::action::ActivateIntervention::Goal goal_msg;
    rclcpp_action::Client<intervention_msgs::action::ActivateIntervention>::SendGoalOptions send_goal_options;
    rclcpp_action::ClientGoalHandle<intervention_msgs::action::ActivateIntervention>::SharedPtr goal_handle;
    rclcpp::Client<intervention_msgs::srv::CompletionConfirmation>::SharedPtr completion_confirmation_client;
    uint8_t priority = 0;
    uint8_t nodeNumber = 0;
    bool finalNode = false;
    std::string uuid;
    sensor_msgs::msg::JointState joint_state;

    std::string error_message;
};