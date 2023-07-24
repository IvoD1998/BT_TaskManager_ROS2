#include "interventions/logger.hpp"


void Logger::logError(const std::shared_ptr<rclcpp::Node> node,
              const std::stringstream &ss)
{
    if(print_ || to_file_)
    {
        std::stringstream output;
        output << "\033[1;31m\n/////////////////////////////////////////////\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << ss.str();
        output << "/////////////////////////////////////////////\033[0m\n";
        if(print_)
        {
            RCLCPP_ERROR(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, ss.str());
        }
    }
}
void Logger::logError(const std::shared_ptr<rclcpp::Node> node,
              const std::string &str)
{
    if(print_ || to_file_)
    {
        std::stringstream output;
        output << "\033[1;31m\n/////////////////////////////////////////////\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << str + "\n";
        output << "/////////////////////////////////////////////\033[0m\n";
        if(print_)
        {
            RCLCPP_ERROR(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, str + "\n");
        }
    }   
}
void Logger::logError(const std::shared_ptr<rclcpp::Node> node,
              const std::stringstream &ss,
              std::string &checkValue)
{
    if((print_ || to_file_) && (ss.str() != checkValue))
    {
        std::stringstream output;
        output << "\033[1;31m\n/////////////////////////////////////////////\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << ss.str();
        output << "/////////////////////////////////////////////\033[0m\n";
        if(print_)
        {    
            RCLCPP_ERROR(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, ss.str());
        }
        checkValue = ss.str();
    }
}
void Logger::logError(const std::shared_ptr<rclcpp::Node> node,
              const std::string &str,
              std::string &checkValue)
{
    if((print_ || to_file_) && (str != checkValue))
    {    
        std::stringstream output;
        output << "\033[1;31m\n/////////////////////////////////////////////\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << str + "\n";
        output << "/////////////////////////////////////////////\033[0m\n";
        if(print_)
        {
            RCLCPP_ERROR(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, str + "\n");
        }
        checkValue = str;
    }
}

void Logger::logWarning(const std::shared_ptr<rclcpp::Node> node,
             const std::stringstream &ss)
{
    if(print_ || to_file_)
    {
        std::stringstream output;
        output << "\033[1;36m\n*********************************\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << ss.str();
        output << "*********************************\033[0m\n";
        if(print_)
        {
            RCLCPP_WARN(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, ss.str());
        }
    }
}
void Logger::logWarning(const std::shared_ptr<rclcpp::Node> node,
             const std::string &str)
{
    if(print_ || to_file_)
    {
        std::stringstream output;
        output << "\033[1;36m\n*********************************\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << str + "\n";
        output << "*********************************\033[0m\n";
        if(print_)
        {
            RCLCPP_WARN(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, str + "\n");
        }
    }
}
void Logger::logWarning(const std::shared_ptr<rclcpp::Node> node,
                const std::stringstream &ss,
                std::string &checkValue)
{
    if((print_ || to_file_) && (ss.str() != checkValue))
    {    
        std::stringstream output;
        output << "\033[1;36m\n*********************************\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << ss.str();
        output << "*********************************\033[0m\n";
        if(print_)
        {
            RCLCPP_WARN(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, ss.str());
        }
        checkValue = ss.str();
    }
}
void Logger::logWarning(const std::shared_ptr<rclcpp::Node> node,
                const std::string &str,
                std::string &checkValue)
{
    if((print_ || to_file_) && (str != checkValue))
    {    
        std::stringstream output;
        output << "\033[1;36m\n*********************************\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << str + "\n";
        output << "*********************************\033[0m\n";
        if(print_)
        {
            RCLCPP_WARN(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, str + "\n");
        }
        checkValue = str;
    }
}

void Logger::logInfo(const std::shared_ptr<rclcpp::Node> node,
             const std::stringstream &ss)
{
    if(print_ || to_file_)
    {
        std::stringstream output;
        output << "\033[1;33m\n---------------------------------\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << ss.str();
        output << "---------------------------------\033[0m\n";
        if(print_)
        {
            RCLCPP_INFO(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, ss.str());
        }
    }
}
void Logger::logInfo(const std::shared_ptr<rclcpp::Node> node,
             const std::string &str)
{
    if(print_ || to_file_)
    {
        std::stringstream output;
        output << "\033[1;33m\n---------------------------------\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << str + "\n";
        output << "---------------------------------\033[0m\n";
        if(print_)
        {
            RCLCPP_INFO(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, str + "\n");
        }
    }
}
void Logger::logInfo(const std::shared_ptr<rclcpp::Node> node,
             const std::stringstream &ss,
             std::string &checkValue)
{
    if((print_ || to_file_) && (ss.str() != checkValue))
    {    
        std::stringstream output;
        output << "\033[1;33m\n---------------------------------\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << ss.str();
        output << "---------------------------------\033[0m\n";
        if(print_)
        {
            RCLCPP_INFO(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, ss.str());
        }
        checkValue = ss.str();
    }
}
void Logger::logInfo(const std::shared_ptr<rclcpp::Node> node,
             const std::string &str,
             std::string &checkValue)
{
    if((print_ || to_file_) && (str != checkValue))
    {    
        std::stringstream output;
        output << "\033[1;33m\n---------------------------------\n";
        output << "From node: '" << node->get_name() << "'\n";
        output << "In namespace: '" << node->get_namespace() << "'\n";
        output << str + "\n";
        output << "---------------------------------\033[0m\n";
        if(print_)
        {
            RCLCPP_INFO(node->get_logger(), output.str().c_str());
        }
        if(to_file_)
        {
            fileWriter(node, str + "\n");
        }
        checkValue = str;
    }
}

void Logger::logFeedback(const std::shared_ptr<rclcpp::Node> node,
             const std::stringstream &ss)
{
    if(print_)
    {
        std::stringstream output;
        output << "\n" << ss.str();
        RCLCPP_INFO(node->get_logger(), output.str().c_str());
    }
}
void Logger::logFeedback(const std::shared_ptr<rclcpp::Node> node,
             const std::string &str)
{
    if(print_)
    {
      std::stringstream output;
        output << str + "\n";
        RCLCPP_INFO(node->get_logger(), output.str().c_str());  
    }
}
void Logger::logFeedback(const std::shared_ptr<rclcpp::Node> node,
                 const std::stringstream &ss,
                 std::string &checkValue)
{
    if((ss.str() != checkValue) && print_)
    {    
        std::stringstream output;
        output << "\n" << ss.str();
        RCLCPP_INFO(node->get_logger(), output.str().c_str());
        checkValue = ss.str();
    }
}
void Logger::logFeedback(const std::shared_ptr<rclcpp::Node> node,
                 const std::string &str,
                 std::string &checkValue)
{
    if((str != checkValue) && print_)
    {    
        std::stringstream output;
        output << str + "\n";
        RCLCPP_INFO(node->get_logger(), output.str().c_str());
    }
}
void Logger::fileWriter(const std::shared_ptr<rclcpp::Node> node, 
                        const std::string str)
{
    if(print_)
    {
        std::ofstream output_log_;
        output_log_.open(name_, std::fstream::app);    
        std::time_t timepoint = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        output_log_ << "--------------------------------------------\n";
        output_log_ << "At: " << std::ctime(&timepoint) << std::endl;
        output_log_ << "From node: " << node->get_name() << std::endl;
        output_log_ << "In namespace: " << node->get_namespace() << std::endl;
        output_log_ << str;
        output_log_ << "--------------------------------------------\n";
        output_log_.close();
    }
}

