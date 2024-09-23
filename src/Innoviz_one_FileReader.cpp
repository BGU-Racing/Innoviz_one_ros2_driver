// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/main/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROS
#include "rclcpp/rclcpp.hpp"

// std
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

// project
#include "ros_utils.h"
#include "InvzRosFileReader.h"

// using
using namespace ros_utils;
using std::cout;
using std::endl;

// constants
static constexpr size_t maxAttributes = 100;

// application entry point
int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<rclcpp::Node>("invz_filereader_publisher");

    // Get parameter for ROS log level (adjust the parameter name as needed)
    int rosLogLevel;
    node->declare_parameter<int>("ros_log_level", 0);
    node->get_parameter("ros_log_level", rosLogLevel);

    // Set ROS log level
    // setRosLoggerLevel(node->get_logger(),rosLogLevel);
    rclcpp::Logger::Level level = (rosLogLevel == 0) ? rclcpp::Logger::Level::Debug :
            (rosLogLevel == 1) ? rclcpp::Logger::Level::Info :
            (rosLogLevel == 2) ? rclcpp::Logger::Level::Warn :
            (rosLogLevel == 3) ? rclcpp::Logger::Level::Error :
            (rosLogLevel == 4) ? rclcpp::Logger::Level::Fatal :
            throw std::runtime_error("failed to set ros logger level!");
    rclcpp::Logger logger = node->get_logger();
    logger.set_level(level);
    // Initialize innoviz ROS file reader
    InvzRosFileReader filereader;

    // Get number of frames
    size_t nofFrames = filereader.getNofFrames();

    // Get fps
    float fps = (nofFrames > 1) ? filereader.getFps() : 1;

    // Log number of frames
    RCLCPP_INFO(logger, "Number of frames in file: %zu", nofFrames);

    // Log fps
    RCLCPP_INFO(logger, "Recording FPS: %f",fps);

    // Log start reading frames
    RCLCPP_INFO(logger, "Start reading frames...");

    // Fixed frame rate
    rclcpp::Rate loop_rate(fps);

    // Read frames while ROS is running - restarting at the end of recording
    for (size_t frame = 0; rclcpp::ok(); frame = (frame + 1) % nofFrames)
    {
        // Grab and handle frames
        filereader.GrabAndPublishFrame(frame, true, true);

        // Sleep necessary time to keep frame rate
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
