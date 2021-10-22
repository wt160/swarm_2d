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
    class FinalCoverageRecorderNode : public rclcpp::Node
    {
    public:
        FinalCoverageRecorderNode(std::string data_filename) : rclcpp::Node{"final_coverage_recorder"}
        {
            using namespace std;
            data_filename_ = data_filename;
            this->declare_parameter("robot_list");
            auto robot_names = this->get_parameter("robot_list").as_string_array();
            
            for (int i = 0; i < robot_names.size() + 1; i++)
            {
                std::string robot_name = "";
                if(i < robot_names.size()){ 
                    robot_name = robot_names[i]; 
                }
                std::function<void(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)> fcn = std::bind(&FinalCoverageRecorderNode::mapCallback, this, std::placeholders::_1, robot_name);
                this->map_sub_list_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(robot_name + "/map", 10, 
                    fcn));
            }

            data_file_.open(data_filename_, std::ofstream::out | std::ofstream::app);
            
            data_acquire_done_ = false;


        }

        

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, std::string robot_name)
        {
            int free_cell_num = 0;
            for(auto v = msg->data.begin(); v != msg->data.end(); v ++)
            {
                if(*v == 0){
                    free_cell_num ++;
                }
            }
            if(data_acquire_done_ == false)
                std::cout<<"area explored by "<<robot_name<<": "<<std::to_string(free_cell_num)<<std::endl;
                
            if(robot_name == ""){
                explored_area_map_["map"] = free_cell_num;
                // std::cout<<"map area:"<<free_cell_num<<std::endl;
            }
            else
            {
                explored_area_map_[robot_name] = free_cell_num;
                // std::cout<<robot_name<<" area:"<<free_cell_num<<std::endl;

            }
            // std::cout<<"map size:"<<explored_area_map_.size()<<std::endl;
            if(explored_area_map_.size() == map_sub_list_.size() && data_acquire_done_ == false){
                data_acquire_done_ = true;
                int area_summation = 0;
                int free_area_total = 0;
                for(auto i = explored_area_map_.begin(); i != explored_area_map_.end(); i ++)
                {
                    if(i->first != "map")
                    {
                        area_summation += i->second;
                    }
                    else
                    {
                        free_area_total = i->second;
                    }
                }
                double coverage_intersection_percent = (double)(area_summation - free_area_total) / free_area_total; 
                std::cout<<"coverage_intersection_percent:"<<coverage_intersection_percent<<std::endl;
                std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
                std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
                std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
                std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
                std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
                std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;

                data_file_ << coverage_intersection_percent << std::endl;
                data_file_.close();
            }
        }



    private:
        std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_sub_list_; 
        std::string data_filename_;
        std::map<std::string, int> explored_area_map_;
        std::ofstream data_file_;
        std::ofstream communication_data_file_;
        bool data_acquire_done_ = false;
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


    auto launcher_ptr = std::make_shared<FinalCoverageRecorderNode>(data_filename);

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(launcher_ptr);

    executor.spin();

    return 0;
}
