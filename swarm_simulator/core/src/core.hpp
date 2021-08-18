#pragma once

#include <map>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <thread>
#include <functional>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <opencv2/opencv.hpp>
#include "state.hpp"

namespace simulator::core
{
    class CoreNode : public rclcpp::Node
    {
    public:
        CoreNode(void);
        void initSimulationEnv(std::vector<std::string>&, std::vector<std::tuple<double, double, double>>&, std::string);
        void setEnvironmentMap(std::string);
        std::pair<int, int> getMapWidthAndHeight(){return std::make_pair(map_width, map_height);}
        double getMapResolution(){return map_resolution;}
        double getTranslationVLimit(){return translation_v_limit;}
        double getRotationVLimit(){return rotation_v_limit;}

        // Core data
        nav_msgs::msg::OccupancyGrid::SharedPtr Map_Ptr;
        std::shared_mutex MapMutex;
        std::shared_ptr<std::map<std::string, State>> States_Ptr;
        std::shared_mutex StatesMutex;
        cv::Mat input_environment_map;
        cv::Mat inverted_input_map;

    private:
        // Simulation parameters
        double iterationTimestep;
        double simualtionFrequency;

        //simulation environemnt parameters
        int map_width;
        int map_height;



        // Visualization parameters
        double visualizationFrequency;
        double poseArrowLength;
        double map_resolution;
        double translation_v_limit;
        double rotation_v_limit;

        rclcpp::TimerBase::SharedPtr simulationTimer_ptr;
        void simulationPeriodElapsedCallback(void);
 
        rclcpp::TimerBase::SharedPtr visualizationTimer_ptr;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr environmentVisualizer_ptr;
        std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> robot_pose_ptr_map;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr statesVisualizer_ptr;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr labelsVisualizer_ptr;
        void visualizationPeriodElapsedCallback(void);
    };
} // namespace simulator::core


