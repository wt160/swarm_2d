#pragma once

#include <map>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>

namespace simulator::core
{
    class CoreNode : rclcpp::Node
    {
        cv::Mat map;
        std::map<std::string, Status> robots;
        timer;

        void periodElapsedCallback()
        {
            // update status
        }
    };
} // namespace simulator::core