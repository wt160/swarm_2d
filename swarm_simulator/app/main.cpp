#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "../plugins/plugins"
#include "../core/core"

int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);

    int robot_num = 10;
    vector<string> robot_names;
    vector<tuple<double, double, double>> robot_init_poses;
    double x_start = 5.0;
    double y_start = 5.0;
    double yaw_start = 0.0; 
    for(int i = 0; i < robot_num; i++){
        robot_names.push_back("tb"+to_string(i));
        robot_init_poses.push_back(make_tuple(x_start, y_start, yaw_start));
        x_start += 5.0;
    }


    shared_ptr<core::CoreNode> core_ptr = make_shared<core::CoreNode>();
    core_ptr->StatesMutex.lock();
    for(int i = 0; i < robot_num; i++){
        core_ptr->States_Ptr->insert(pair<string, core::State>{robot_names[i], core::State{.X = get<0>(robot_init_poses[i]), .Y = get<1>(robot_init_poses[i]), .Dir = get<2>(robot_init_poses[i]), .VX = 0.0, .VY = 0.0, .W = 0.0}});
    }
    
    core_ptr->StatesMutex.unlock();
    core_ptr->setEnvironmentMap("/home/wei/swarm_2d_ws/src/swarm_simulator/app/warehouse_01.png");

    vector<shared_ptr<plugin::FieldOfView>> fov_list;
    vector<shared_ptr<plugin::NavigationPlugin>> nav_list;
    for(int i = 0; i < robot_num; i++)
    {
        fov_list.push_back(make_shared<plugin::FieldOfView>(robot_names[i], 5, 100, core_ptr));
        nav_list.push_back(make_shared<plugin::NavigationPlugin>(robot_names[i], core_ptr));
    }
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(core_ptr);
    for(int i = 0; i < robot_num; i++)
    {
        executor.add_node(fov_list[i]);
        executor.add_node(nav_list[i]);
    }

    executor.spin();

    return 0;
}
