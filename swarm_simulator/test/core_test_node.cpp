#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "../core/core"

int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);

    shared_ptr<core::CoreNode> core_ptr = make_shared<core::CoreNode>();
    core_ptr->StatesMutex.lock();
    core_ptr->States_Ptr->insert(pair<string, core::State>{"test1", core::State{.X = 0, .Y = 0, .Dir = 0, .VX = 0.2, .VY = 0.5, .W = 0.1}});
    core_ptr->States_Ptr->insert(pair<string, core::State>{"test2", core::State{.X = 0, .Y = 0, .Dir = 0, .VX = -0.2, .VY = 0.1, .W = -0.2}});
    core_ptr->StatesMutex.unlock();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(core_ptr);
    executor.spin();

    return 0;
}
