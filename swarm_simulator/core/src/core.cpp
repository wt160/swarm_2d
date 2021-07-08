#include "core.hpp"

#include <cmath>

#include "geometry_msgs/msg/point.hpp"

namespace simulator::core
{
    CoreNode::CoreNode(void) : rclcpp::Node{"simulator_core_node"}
    {
        this->Map_Ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        this->States_Ptr = std::make_shared<std::map<std::string, State>>();

        this->declare_parameter("iteration_timestep", 0.01);
        this->declare_parameter("simulation_frequency", 100.0);

        this->declare_parameter("visualization_frequency", 30.0);
        this->declare_parameter("pose_array_length", 1.0);

        this->iterationTimestep = this->get_parameter("iteration_timestep").as_double();
        this->simualtionFrequency = this->get_parameter("simulation_frequency").as_double();
        this->visualizationFrequency = this->get_parameter("visualization_frequency").as_double();
        this->poseArrowLength = this->get_parameter("pose_array_length").as_double();

        this->environmentVisualizer_ptr = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        this->statesVisualizer_ptr = this->create_publisher<visualization_msgs::msg::MarkerArray>("states", 10);
        this->labelsVisualizer_ptr = this->create_publisher<visualization_msgs::msg::MarkerArray>("labels", 10);

        this->simulationTimer_ptr = this->create_wall_timer(std::chrono::microseconds((int)(1000000 / this->simualtionFrequency)),
                                                            std::bind(&CoreNode::simulationPeriodElapsedCallback, this));
        this->visualizationTimer_ptr = this->create_wall_timer(std::chrono::microseconds((int)(1000000 / this->visualizationFrequency)),
                                                               std::bind(&CoreNode::visualizationPeriodElapsedCallback, this));
    }

    void CoreNode::simulationPeriodElapsedCallback(void)
    {
        this->StatesMutex.lock();
        for (auto &&pair : *this->States_Ptr)
        {
            State &state = (*this->States_Ptr)[pair.first];
            state.X += state.VX * this->iterationTimestep;
            state.Y += state.VY * this->iterationTimestep;
            state.Dir += state.W * this->iterationTimestep;
            while (state.Dir > M_PI)
                state.Dir -= M_PI * 2;
            while (state.Dir <= -M_PI)
                state.Dir += M_PI * 2;
        }
        this->StatesMutex.unlock();
    }

    void CoreNode::visualizationPeriodElapsedCallback(void)
    {
        using namespace visualization_msgs::msg;
        using namespace geometry_msgs::msg;

        this->MapMutex.lock_shared();
        this->environmentVisualizer_ptr->publish(*this->Map_Ptr);
        this->MapMutex.unlock_shared();

        this->StatesMutex.lock_shared();
        MarkerArray poses, labels;
        int id = 0;
        for (auto &&pair : *this->States_Ptr)
        {
            State &state = (*this->States_Ptr)[pair.first];

            Marker pose;
            pose.id = id;
            pose.header.frame_id = "map", pose.header.stamp = this->get_clock()->now();
            pose.type = Marker::ARROW;
            pose.color.a = 100, pose.color.r = 100;
            pose.scale.x = 0.1, pose.scale.y = 0.2;
            Point head, tail;
            head.x = state.X, head.y = state.Y, head.z = 0;
            tail.x = head.x + this->poseArrowLength * std::cos(state.Dir);
            tail.y = head.y + this->poseArrowLength * std::sin(state.Dir);
            tail.z = 0;
            pose.points.emplace_back(head);
            pose.points.emplace_back(tail);
            poses.markers.push_back(pose);

            Marker label;
            label.id = id;
            label.header.frame_id = "map", label.header.stamp = this->get_clock()->now();
            label.type = Marker::TEXT_VIEW_FACING;
            label.color.a = 100;
            label.text = pair.first;
            label.scale.z = 0.3;
            label.pose.position.x = state.X, label.pose.position.y = state.Y, label.pose.position.z = 0.5;
            labels.markers.push_back(label);

            ++id;
        }
        this->statesVisualizer_ptr->publish(poses);
        this->labelsVisualizer_ptr->publish(labels);
        this->StatesMutex.unlock_shared();
    }
} // namespace simulator::core
