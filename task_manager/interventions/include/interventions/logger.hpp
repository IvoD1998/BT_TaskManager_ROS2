#pragma once

#include "rclcpp/rclcpp.hpp"
#include <fstream>

class Logger
{
    public:
    Logger(const bool print = true, const bool to_file = false)
    {
        print_ = print;
        to_file_ = to_file;
        if(to_file_)
        {
            std::time_t timepoint = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::stringstream ss;
            ss << "logfile_" << std::ctime(&timepoint) << ".txt";
            name_ = "/home/ivo/testlogs/" + ss.str();
        }
    }
    ~Logger()
    {
    }

    /**
     * @brief Log message with error level severity
     * @param node is the node where the log message occurs
     * @param ss is the log message in stringstream form
    */
    void logError(const std::shared_ptr<rclcpp::Node> node,
                const std::stringstream &ss);

    /**
     * @brief Log message with error level severity
     * @param node is the node where the log message occurs
     * @param str is the log message in string form
    */
    void logError(const std::shared_ptr<rclcpp::Node> node,
                const std::string &str);


    void logError(const std::shared_ptr<rclcpp::Node> node,
                const std::stringstream &ss,
                std::string &checkValue);

    void logError(const std::shared_ptr<rclcpp::Node> node,
                const std::string &str,
                std::string &checkValue);

    /**
     * @brief Log message with warning level severity
     * @param node is the node where the log message occurs
     * @param str is the log message in stringstream form
    */        
    void logWarning(const std::shared_ptr<rclcpp::Node> node,
                    const std::stringstream &ss);
    /**
     * @brief Log message with warning level severity
     * @param node is the node where the log message occurs
     * @param str is the log message in string form
    */
    void logWarning(const std::shared_ptr<rclcpp::Node> node,
                    const std::string &str);
    void logWarning(const std::shared_ptr<rclcpp::Node> node,
                    const std::stringstream &ss,
                    std::string &checkValue);
    void logWarning(const std::shared_ptr<rclcpp::Node> node,
                    const std::string &str,
                    std::string &checkValue);

    /**
     * @brief Log message with info level severity
     * @param node is the node where the log message occurs
     * @param str is the log message in stringstream form
    */    
    void logInfo(const std::shared_ptr<rclcpp::Node> node,
                const std::stringstream &ss);
    /**
     * @brief Log message with info level severity
     * @param node is the node where the log message occurs
     * @param str is the log message in string form
    */
    void logInfo(const std::shared_ptr<rclcpp::Node> node,
                const std::string &str);
    void logInfo(const std::shared_ptr<rclcpp::Node> node,
                const std::stringstream &ss,
                std::string &checkValue);
    void logInfo(const std::shared_ptr<rclcpp::Node> node,
                const std::string &str,
                std::string &checkValue);

    /**
     * @brief Log message with info level severity specifically for action feedback
     * @param node is the node where the log message occurs
     * @param str is the log message in stringstream form
    */    
    void logFeedback(const std::shared_ptr<rclcpp::Node> node,
                    const std::stringstream &ss);
    /**
     * @brief Log message with info level severity specifically for action feedback
     * @param node is the node where the log message occurs
     * @param str is the log message in string form
    */
    void logFeedback(const std::shared_ptr<rclcpp::Node> node,
                    const std::string &str);
    void logFeedback(const std::shared_ptr<rclcpp::Node> node,
                    const std::stringstream &ss,
                    std::string &checkValue);
    void logFeedback(const std::shared_ptr<rclcpp::Node> node,
                    const std::string &str,
                    std::string &checkValue);
    void fileWriter(const std::shared_ptr<rclcpp::Node> node, 
                    const std::string str);
private:
    bool print_, to_file_;
    std::string name_;
};