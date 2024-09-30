#include "ros_utils.h"

namespace ros_utils{
    void setRosLoggerLevel(rclcpp::Logger& logger,int loggerLevel){
        rclcpp::Logger::Level level;
        switch (loggerLevel){
            case 0:
                level = rclcpp::Logger::Level::Debug;
                break;
            case 1:
                level = rclcpp::Logger::Level::Info;
                break;
            case 2:
                level = rclcpp::Logger::Level::Warn;
                break;
            case 3:
                level = rclcpp::Logger::Level::Error;
                break;
            case 4:
                level = rclcpp::Logger::Level::Fatal;
                break;
            default:
                throw std::runtime_error("failed to set ros logger level!");
        }
        logger.set_level(level);
    }
}