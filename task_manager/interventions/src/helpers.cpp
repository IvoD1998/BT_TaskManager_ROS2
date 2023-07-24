#include "interventions/plunger_intervention/helpers.hpp"

Eigen::MatrixXd poseToHomMat(geometry_msgs::msg::PoseStamped &pose)
{
    double x, y, z, w;
    x = pose.pose.orientation.x;
    y = pose.pose.orientation.y;
    z = pose.pose.orientation.z;
    w = pose.pose.orientation.w;

    Eigen::MatrixXd output(4, 4);
    output << (1 - 2*y*y - 2*z*z), (2*x*y - 2*w*z), (2*x*z + 2*w*y), pose.pose.position.x,
              (2*x*y + 2*w*z), (1 - 2*x*x - 2*z*z), (2*y*z - 2*w*x), pose.pose.position.y,
              (2*x*z - 2*w*y), (2*y*z + 2*w*x), (1 - 2*x*x - 2*y*y), pose.pose.position.z, 
              0, 0, 0, 1;
    return output;
}
Eigen::MatrixXd transformToHomMat(geometry_msgs::msg::TransformStamped &trans)
{
    double x, y, z, w;
    x = trans.transform.rotation.x;
    y = trans.transform.rotation.y;
    z = trans.transform.rotation.z;
    w = trans.transform.rotation.w;

    Eigen::MatrixXd output(4, 4);
    output << (1 - 2*y*y - 2*z*z), (2*x*y - 2*w*z), (2*x*z + 2*w*y), trans.transform.translation.x,
              (2*x*y + 2*w*z), (1 - 2*x*x - 2*z*z), (2*y*z - 2*w*x), trans.transform.translation.y,
              (2*x*z - 2*w*y), (2*y*z + 2*w*x), (1 - 2*x*x - 2*y*y), trans.transform.translation.z, 
              0, 0, 0, 1;
    return output;
}
Eigen::MatrixXd invertMatrix(Eigen::MatrixXd &matrix)
{
    Eigen::MatrixXd output(4, 4);
    output = matrix;
    output.block<3, 3>(0, 0) = matrix.block<3, 3>(0, 0).inverse();
    output.block<3, 1>(0, 3) = -matrix.block<3, 3>(0, 0).inverse() * matrix.block<3, 1>(0, 3);
    return output;
}
geometry_msgs::msg::Pose homMatToPose(Eigen::MatrixXd &m)
{
    geometry_msgs::msg::Pose output;
    output.position.x = m(0, 3);
    output.position.y = m(1, 3);
    output.position.z = m(2, 3);
    float trace = m(0, 0) + m(1, 1) + m(2, 2);
    if(trace > 0)
    {
        float s = 0.5f / sqrtf(trace + 1.0f);
        output.orientation.w = 0.25f / s;
        output.orientation.x = (m(2, 1) - m(1, 2)) * s;
        output.orientation.y = (m(0, 2) - m(2, 0)) * s;
        output.orientation.z = (m(1, 0) - m(0, 1)) * s;
    }
    else
    {
        if((m(0, 0) > m(1, 1)) && (m(0, 0) > m(2, 2)))
        {
            float s = 2.0f * sqrtf(1.0f + m(0, 0) - m(1, 1) - m(2, 2));
            output.orientation.w = (m(2, 1) - m(1, 2)) / s;
            output.orientation.x = 0.25f * s;
            output.orientation.y = (m(0, 1) + m(1, 0)) / s;
            output.orientation.z = (m(0, 2) + m(2, 0)) / s;
        }
        else if(m(1, 1) > m(2, 2))
        {
            float s = 2.0f * sqrtf(1.0f + m(1, 1) - m(0, 0) - m(2, 2));
            output.orientation.w = (m(0, 2) - m(2, 0)) / s;
            output.orientation.x = (m(0, 1) + m(1, 0)) / s;
            output.orientation.y = 0.25f * s;
            output.orientation.z = (m(1, 2) + m(2, 1)) / s;
        }
        else
        {
            float s = 2.0f * sqrtf(1.0f + m(2, 2) - m(0, 0) - m(1, 1));
            output.orientation.w = (m(1, 0) - m(0, 1)) / s;
            output.orientation.x = (m(0, 2) + m(2, 0)) / s;
            output.orientation.y = (m(1, 2) + m(2, 1)) / s;
            output.orientation.z = 0.25f * s;
        }
    }
    return output;
}
