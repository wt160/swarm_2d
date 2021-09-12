#include "navigation.hpp" 


namespace simulator::plugin
{
    NavigationPlugin::NavigationPlugin(std::string robot_name, std::shared_ptr<simulator::core::CoreNode>& core_ptr) : rclcpp::Node{"navigation_plugin_" + robot_name}
    {
        core_ptr_ = core_ptr;
        robot_name_ = robot_name;
        shared_map_ptr = core_ptr->Map_Ptr;
        //navigation_action_callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
        this->navigation_action_server_ = rclcpp_action::create_server<NavigationAction>(this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(), this->get_node_waitables_interface(), robot_name_ + "/navigation_action", std::bind(&NavigationPlugin::handle_action_goal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&NavigationPlugin::handle_action_cancel, this, std::placeholders::_1), std::bind(&NavigationPlugin::handle_action_accepted, this, std::placeholders::_1), rcl_action_server_get_default_options());
        this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(robot_name_ + "/cmd_vel", 10, std::bind(&NavigationPlugin::cmdVelCallback, this, std::placeholders::_1));
        path_visualize_ptr = this->create_publisher<visualization_msgs::msg::Marker>(robot_name_ + "/plan_path", 10);
    }

    void NavigationPlugin::publishPlannedPath(std::list<Point>& path, double resolution)
    {
        visualization_msgs::msg::Marker path_marker;
        path_marker.id = 0;
        path_marker.header.frame_id = "map", path_marker.header.stamp = this->get_clock()->now();
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.color.a = 100, path_marker.color.r = 100;
        path_marker.scale.x = 0.1, path_marker.scale.y = 0.2;
        path_marker.action = 0;

        for(auto p = path.begin(); p != path.end(); p ++)
        {
            geometry_msgs::msg::Point geo_p;
            geo_p.x = p->x * resolution;
            geo_p.y = p->y * resolution;
            geo_p.z = 0;
            path_marker.points.push_back(geo_p);

        }
        path_visualize_ptr->publish(path_marker);
    }

    void NavigationPlugin::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::cout<<robot_name_<<" cmdvel callback"<<std::endl;
        (*(core_ptr_->States_Ptr))[robot_name_].VX = msg->linear.x;
        (*(core_ptr_->States_Ptr))[robot_name_].VY = msg->linear.y;
        (*(core_ptr_->States_Ptr))[robot_name_].W = msg->angular.z;


    }

    rclcpp_action::GoalResponse NavigationPlugin::handle_action_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigationAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        // Let's reject sequences that are over 9000
        // if (goal->order > 9000) {
        //   return rclcpp_action::GoalResponse::REJECT;
        // }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse NavigationPlugin::handle_action_cancel(
        const std::shared_ptr<GoalHandleNavigationAction> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void NavigationPlugin::execute(const std::shared_ptr<GoalHandleNavigationAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigationAction::Feedback>();
        // auto & sequence = feedback->sequence;
        // sequence.push_back(0);
        // sequence.push_back(1);
        auto result = std::make_shared<NavigationAction::Result>();
        double map_resolution = core_ptr_->getMapResolution();
    
        geometry_msgs::msg::Pose current_robot_pose_local_frame_ = goal->target_pose_local_frame.pose;
        // peer_pose_in_peer_frame_dict_ = peer_pose_in_peer_frame_dict;
        //get current robot's current pose from simulator::core
        simulator::core::State curr_state = (*(core_ptr_->States_Ptr))[robot_name_];
        double curr_x = curr_state.X;
        double curr_y = curr_state.Y;
        double curr_theta = curr_state.Dir;

        int curr_x_coord = (round)(curr_x /map_resolution); 
        int curr_y_coord = (round)(curr_y /map_resolution); 

        int target_x_coord = (round)(current_robot_pose_local_frame_.position.x / map_resolution);
        int target_y_coord = (round)(current_robot_pose_local_frame_.position.y / map_resolution);

        //call pathfinder to calculate the path
        PathFinder path_finder;
        path_finder.init(shared_map_ptr);
        Point start(curr_x_coord, curr_y_coord);
        Point end(target_x_coord, target_y_coord); 
        if(path_finder.search(start, end)){
            //success find path from start to end
            std::list<Point> path;
            std::cout<<"success"<<std::endl;
            double path_cost = path_finder.getPathAndCost(path);
            std::cout<<"path cost:"<<path_cost<<std::endl;
            publishPlannedPath(path, map_resolution);
            Point curr_navigation_pt = start;
            auto path_iterator = path.begin();
            path_iterator ++;
            Point next_navigation_pt = *path_iterator;
            double next_navigation_x = next_navigation_pt.x *map_resolution;
            double next_navigation_y = next_navigation_pt.y *map_resolution;

            double robot_pos_x = (double)start.x* map_resolution;
            double robot_pos_y = (double)start.y* map_resolution;
            double robot_pos_theta = 0.0;
            double end_pos_x = (double)end.x * map_resolution;
            double end_pos_y = (double)end.y * map_resolution;

            double v_limit = core_ptr_->getTranslationVLimit();
            double w_limit = core_ptr_->getRotationVLimit();

            double vx = 0.0;
            double vy = 0.0;
            double vw = 0.0;
            while(std::fabs(robot_pos_x - end_pos_x) > map_resolution || std::fabs(robot_pos_y - end_pos_y) > map_resolution){
                
                while(std::fabs(robot_pos_x - next_navigation_x) > 0.3*v_limit*map_resolution || std::fabs(robot_pos_y - next_navigation_y) > 0.3*v_limit*map_resolution){ 
                    //std::cout<<"x_diff:"<<robot_pos_x - next_navigation_x<<", y_diff:"<<robot_pos_y - next_navigation_y<<std::endl;
                    robot_pos_x = (*(core_ptr_->States_Ptr))[robot_name_].X;
                    robot_pos_y = (*(core_ptr_->States_Ptr))[robot_name_].Y;
                    robot_pos_theta = (*(core_ptr_->States_Ptr))[robot_name_].Dir;
                    double dx = next_navigation_x - robot_pos_x;
                    double dy = next_navigation_y - robot_pos_y;
                    double v_length = std::sqrt(dx*dx + dy*dy);
                    vx = v_limit * dx / v_length;
                    vy = v_limit * dy / v_length;
                    vw = 0.1;
                    //std::cout<<"vx,vy,vw:"<<vx<<","<<vy<<","<<vw<<std::endl;
                    (*(core_ptr_->States_Ptr))[robot_name_].VX = vx;
                    (*(core_ptr_->States_Ptr))[robot_name_].VY = vy;
                    (*(core_ptr_->States_Ptr))[robot_name_].W = vw;
                }
                // path_iterator ++;
                path_iterator ++;

                next_navigation_pt = *path_iterator;
                next_navigation_x = next_navigation_pt.x *map_resolution;
                next_navigation_y = next_navigation_pt.y *map_resolution;


            }

            (*(core_ptr_->States_Ptr))[robot_name_].VX = 0.0;
            (*(core_ptr_->States_Ptr))[robot_name_].VY = 0.0;
            (*(core_ptr_->States_Ptr))[robot_name_].W = 0.0;
            

        }else{
            //fail, why?
        }
        





        //iterate over between calculate the velocity and update the pos in simulator::core, until reach the target pose




        // result->current_target_pose = curr_target_pose_local_frame;
        result->result = 1;
        goal_handle->succeed(result);
                    // return curr_target_pose_local_frame;



        
        

        // Check if goal is done
        if (rclcpp::ok()) {
        //   result->sequence = sequence;
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
    }

    void NavigationPlugin::handle_action_accepted(const std::shared_ptr<GoalHandleNavigationAction> goal_handle)
    {
        // using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&NavigationPlugin::execute, this, std::placeholders::_1), goal_handle}.detach();
            // execute(goal_handle);
    }

}