#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "../plugins/plugins"
#include "../core/core"

namespace simulator
{
    class ExperimentRecorderNode : public rclcpp::Node
    {
    public:
        ExperimentRecorderNode() : rclcpp::Node{"experiment_recorder"}
        {
            using namespace std;

            // this->declare_parameter("map");
            this->declare_parameter("names");
            this->declare_parameter("xs");
            this->declare_parameter("ys");
            this->declare_parameter("zs");
            this->declare_parameter("range", 200.0);
            this->declare_parameter("slam_freq", 5.0);
            this->declare_parameter("map_data_filename");
            this->declare_parameter("communication_data_filename");

            // auto map_file = this->get_parameter("map").as_string();
            auto names = this->get_parameter("names").as_string_array();
            auto xs = this->get_parameter("xs").as_double_array();
            auto ys = this->get_parameter("ys").as_double_array();
            auto zs = this->get_parameter("zs").as_double_array();
            auto range = this->get_parameter("range").as_double();
            auto slam_freq = this->get_parameter("slam_freq").as_double();
            auto map_data_filename = this->get_parameter("map_data_filename").as_string();



            int num = min({names.size(), xs.size(), ys.size(), zs.size()});

            vector<string> robot_names{names.begin(), names.begin() + num};
            vector<tuple<double, double, double>> robot_poses;
            for (int i = 0; i < num; i++)
                robot_poses.push_back({xs[i], ys[i], zs[i]});

            this->map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&ExperimentRecorderNode::mapCallback, this, std::placeholders::_1));

            map_data_file_.open (map_data_filename);
            // map_data_file_ << "Writing this to a file.\n";
            // map_data_file_.close();



        }

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            int free_cell_num = 0;
            for(auto v = msg->data.begin(); v != msg->data.end(); v ++)
            {
                if(*v == 0){
                    free_cell_num ++;
                }
            }
            map_data_file_ <<std::to_string(free_cell_num)<<"\n";

        }


 
    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; 
        std::ofstream map_data_file_;
        std::ofstream communication_data_file_;
    };
} // namespace simulator

int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);

    auto launcher_ptr = std::make_shared<ExperimentRecorderNode>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(launcher_ptr);

    executor.spin();

    return 0;
}
