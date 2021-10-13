#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "../plugins/plugins"
#include "../core/core"

namespace simulator
{
    class LauncherNode : public rclcpp::Node
    {
    public:
        LauncherNode() : rclcpp::Node{"simulator_mainapp"}
        {
            using namespace std;

            this->declare_parameter("map");
            this->declare_parameter("names");
            this->declare_parameter("xs");
            this->declare_parameter("ys");
            this->declare_parameter("zs");
            this->declare_parameter("range", 200.0);
            this->declare_parameter("slam_freq", 5.0);

            auto map_file = this->get_parameter("map").as_string();
            auto names = this->get_parameter("names").as_string_array();
            auto xs = this->get_parameter("xs").as_double_array();
            auto ys = this->get_parameter("ys").as_double_array();
            auto zs = this->get_parameter("zs").as_double_array();
            auto range = this->get_parameter("range").as_double();
            auto slam_freq = this->get_parameter("slam_freq").as_double();

            int num = min({names.size(), xs.size(), ys.size(), zs.size()});

            vector<string> robot_names{names.begin(), names.begin() + num};
            vector<tuple<double, double, double>> robot_poses;
            for (int i = 0; i < num; i++)
                robot_poses.push_back({xs[i], ys[i], zs[i]});

            this->core_ptr->initSimulationEnv(robot_names, robot_poses, map_file);

            for (int i = 0; i < num; i++)
            {
                this->fov_list.push_back(std::make_shared<plugin::FieldOfView>(robot_names[i], slam_freq, range, this->core_ptr));
                this->nav_list.push_back(std::make_shared<plugin::NavigationPlugin>(robot_names[i], this->core_ptr));
            }
        }

        std::vector<rclcpp::Node::SharedPtr> GetNodes(void)
        {
            std::vector<rclcpp::Node::SharedPtr> nodes;
            nodes.push_back(this->core_ptr);
            for (auto &&fov : this->fov_list)
                nodes.push_back(fov);
            for (auto &&nav : this->nav_list)
                nodes.push_back(nav);
            return nodes;
        }

    private:
        std::shared_ptr<core::CoreNode> core_ptr = std::make_shared<core::CoreNode>();
        std::vector<std::shared_ptr<plugin::FieldOfView>> fov_list;
        std::vector<std::shared_ptr<plugin::NavigationPlugin>> nav_list;
    };
} // namespace simulator

int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);

    auto launcher_ptr = std::make_shared<LauncherNode>();

    rclcpp::executors::SingleThreadedExecutor executor;

    for (auto &&node_ptr : launcher_ptr->GetNodes())
        executor.add_node(node_ptr);

    executor.spin();

    return 0;
}
