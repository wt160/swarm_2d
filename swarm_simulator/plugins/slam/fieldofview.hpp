#pragma once
#include <vector>
#include <unordered_map>
#include <cmath>
#include <queue>
#include <list>
#include <algorithm>
#include <functional>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "../../core/src/core.hpp"
#include "../../core/src/state.hpp"
#define SGN(a) (((a)<0) ? -1 : 1)

namespace simulator::plugin
{
    
    using namespace std;
    class FieldOfView: public rclcpp::Node
    {
        public:
            FieldOfView(std::string robot_name, double simulate_frequency, int sight_length, std::shared_ptr<simulator::core::CoreNode>& core_ptr);
            vector<pair<int, int>> getLineOfSight(int monster_x, int monster_y);
            void initMapWithUnknownMask();
            bool isValidMapPoint(int coord_x, int coord_y);
            void simulateSLAM();

            // void setStart(Point);
            // void setEnd(Point);

        private:
            std::string robot_name_;
            int map_width;
            int map_height;
            nav_msgs::msg::OccupancyGrid::SharedPtr robot_map_ptr;
            nav_msgs::msg::OccupancyGrid::SharedPtr shared_map_ptr;
            rclcpp::TimerBase::SharedPtr field_of_view_ptr;
            double field_of_view_frequency;
            // Map m; 
            set<pair<int, int>> center_to_circle_set;
            set<vector<pair<int, int>>> check_line_list;
            int line_of_sight_length;
            std::shared_ptr<simulator::core::CoreNode> core_ptr_;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr robot_map_pub_ptr;
    };
}