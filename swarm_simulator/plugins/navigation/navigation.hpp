#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "swarm_interfaces/action/navigation_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "../../core/src/core.hpp"
#include "../../core/src/state.hpp"
#include "pathfinder.hpp"

namespace simulator::plugin
{
    class NavigationPlugin : public rclcpp::Node
    {
    public:
        using NavigationAction = swarm_interfaces::action::NavigationAction;
        using GoalHandleNavigationAction = rclcpp_action::ServerGoalHandle<NavigationAction>;

        NavigationPlugin(std::string, std::shared_ptr<simulator::core::CoreNode> &);
        rclcpp_action::GoalResponse handle_action_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigationAction::Goal> goal);
        rclcpp_action::CancelResponse handle_action_cancel(const std::shared_ptr<GoalHandleNavigationAction> goal_handle);
        void execute(const std::shared_ptr<GoalHandleNavigationAction> goal_handle);
        void handle_action_accepted(const std::shared_ptr<GoalHandleNavigationAction> goal_handle);
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void publishPlannedPath(std::list<Point> &path, double);

        std::shared_ptr<simulator::core::CoreNode> core_ptr_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    private:
        std::string robot_name_;
        bool got_new_goal_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_visualize_ptr;
        nav_msgs::msg::OccupancyGrid::SharedPtr shared_map_ptr;
        rclcpp_action::Server<NavigationAction>::SharedPtr navigation_action_server_;
        rclcpp::callback_group::CallbackGroup::SharedPtr navigation_action_callback_group_;
    };
} // namespace simulator::plugin
