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
        ExperimentRecorderNode(std::string data_filename) : rclcpp::Node{"experiment_recorder"}
        {
            using namespace std;
	    this->data_filename_ = data_filename;

            this->map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&ExperimentRecorderNode::mapCallback, this, std::placeholders::_1));

            map_data_file_.open (this->data_filename_,std::ofstream::out);
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
	std::string data_filename_;
    };
} // namespace simulator

int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);

    std::string data_filename = "";
    if(argc > 1){
        data_filename = std::string(argv[1]);
    }


    auto launcher_ptr = std::make_shared<ExperimentRecorderNode>(data_filename);


    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(launcher_ptr);

    executor.spin();

    return 0;
}
