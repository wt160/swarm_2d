#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "../plugins/plugins"
#include "../core/core"
#include "swarm_interfaces/action/navigation_action.hpp"

int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);
    

    shared_ptr<core::CoreNode> core_ptr = make_shared<core::CoreNode>();
    core_ptr->StatesMutex.lock();
    
    
    core_ptr->States_Ptr->insert(pair<string, core::State>{"tb0", core::State{.X = 1, .Y = 1, .Dir = 0, .VX = 0.0, .VY = 0.0, .W = 0.0}});
    core_ptr->States_Ptr->insert(pair<string, core::State>{"tb1", core::State{.X = 5, .Y = 5, .Dir = 0, .VX = 0.0, .VY = 0.0, .W = -0.0}});
    
    core_ptr->StatesMutex.unlock();
    core_ptr->setEnvironmentMap("/home/wei/swarm_2d_ws/src/swarm_simulator/test/Drawing.png");

    shared_ptr<plugin::FieldOfView> fov_0 = make_shared<plugin::FieldOfView>("tb0", 20, 100, core_ptr);
    shared_ptr<plugin::FieldOfView> fov_1 = make_shared<plugin::FieldOfView>("tb1", 20, 100, core_ptr);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(core_ptr);
    executor.add_node(fov_0);
    executor.add_node(fov_1);

    executor.spin();

    return 0;
}
