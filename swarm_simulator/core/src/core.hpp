#pragma once

#include <map>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <thread>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "state.hpp"

namespace simulator::core
{
    class CoreNode : public rclcpp::Node
    {
    public:
        CoreNode(void);

        // Core data
        nav_msgs::msg::OccupancyGrid::SharedPtr Map_Ptr;
        std::shared_mutex MapMutex;
        std::shared_ptr<std::map<std::string, State>> States_Ptr;
        std::shared_mutex StatesMutex;

    private:
        // Simulation parameters
        double iterationTimestep;
        double simualtionFrequency;

        // Visualization parameters
        double visualizationFrequency;
        double poseArrowLength;

        rclcpp::TimerBase::SharedPtr simulationTimer_ptr;
        void simulationPeriodElapsedCallback(void);

        rclcpp::TimerBase::SharedPtr visualizationTimer_ptr;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr environmentVisualizer_ptr;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr statesVisualizer_ptr;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr labelsVisualizer_ptr;
        void visualizationPeriodElapsedCallback(void);
    };
} // namespace simulator::core