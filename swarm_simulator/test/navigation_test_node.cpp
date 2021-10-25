#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "../plugins/plugins"
#include "../core/core"
#include "rclcpp_action/rclcpp_action.hpp"
#include "swarm_interfaces/action/navigation_action.hpp"

class NavigationActionClient : public rclcpp::Node
{
public:
  using NavigationAction = swarm_interfaces::action::NavigationAction;
  using GoalHandleNavigationAction = rclcpp_action::ClientGoalHandle<NavigationAction>;

  NavigationActionClient(std::string robot_name)
  : Node("navigation_action_client_" + robot_name)
  {
    robot_name_ = robot_name;
    this->client_ptr_ = rclcpp_action::create_client<NavigationAction>(
      this, robot_name_ + 
      "/navigation_action");

  }

  void send_goal(geometry_msgs::msg::PoseStamped target_pose)
  {
    using namespace std::placeholders;


    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigationAction::Goal();
    goal_msg.target_pose_local_frame = target_pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
  
    auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigationActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavigationActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavigationActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigationAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string robot_name_;

  void goal_response_callback(std::shared_future<GoalHandleNavigationAction::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigationAction::SharedPtr,
    const std::shared_ptr<const NavigationAction::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleNavigationAction::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    // rclcpp::shutdown();
  }
};  // class FibonacciActionClient



int main(int argc, char const *argv[])
{
    using namespace std;
    using namespace simulator;

    rclcpp::init(argc, argv);
    double target_x = std::stod(argv[1]);
    double target_y = std::stod(argv[2]);

    std::string robot_name = "robot01";
    shared_ptr<NavigationActionClient> nav_ptr = make_shared<NavigationActionClient>(robot_name);
    geometry_msgs::msg::PoseStamped  goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = target_x;
    goal.pose.position.y = target_y;

    nav_ptr->send_goal(goal);
   
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nav_ptr);

    executor.spin();

    return 0;
}