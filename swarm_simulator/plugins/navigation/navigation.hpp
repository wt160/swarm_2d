#pragma once

namespace simulator::plugin
{
    class Navigation : rclcpp::Node
    {
        std::shared_ptr<SimulatorCore> core_ptr;
    };
} // namespace simulator::plugin
