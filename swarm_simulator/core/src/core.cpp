#include "core.hpp"

#include <cmath>

#include "geometry_msgs/msg/point.hpp"

namespace simulator::core
{
    CoreNode::CoreNode(void) : rclcpp::Node{"simulator_core_node"}
    {
        this->Map_Ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        this->States_Ptr = std::make_shared<std::map<std::string, State>>();

        this->declare_parameter("iteration_timestep", 0.02);
        this->declare_parameter("simulation_frequency", 50.0);

        this->declare_parameter("visualization_frequency", 30.0);
        this->declare_parameter("pose_array_length", 1.0);
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("translation_v_limit", 1.0);
        this->declare_parameter("rotation_v_limit", 1.0);

        this->iterationTimestep = this->get_parameter("iteration_timestep").as_double();
        this->simualtionFrequency = this->get_parameter("simulation_frequency").as_double();
        this->visualizationFrequency = this->get_parameter("visualization_frequency").as_double();
        this->poseArrowLength = this->get_parameter("pose_array_length").as_double();
        this->map_resolution = this->get_parameter("map_resolution").as_double();
        this->translation_v_limit = this->get_parameter("translation_v_limit").as_double();
        this->rotation_v_limit = this->get_parameter("rotation_v_limit").as_double();


        this->environmentVisualizer_ptr = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        this->statesVisualizer_ptr = this->create_publisher<visualization_msgs::msg::MarkerArray>("states", 10);
        this->labelsVisualizer_ptr = this->create_publisher<visualization_msgs::msg::MarkerArray>("labels", 10);

        this->simulationTimer_ptr = this->create_wall_timer(std::chrono::microseconds((int)(1000000 / this->simualtionFrequency)),
                                                            std::bind(&CoreNode::simulationPeriodElapsedCallback, this));
        this->visualizationTimer_ptr = this->create_wall_timer(std::chrono::microseconds((int)(1000000 / this->visualizationFrequency)),
                                                               std::bind(&CoreNode::visualizationPeriodElapsedCallback, this));


        
    }
    void CoreNode::initSimulationEnv(std::vector<std::string>& robot_names, std::vector<std::tuple<double, double, double>>& robot_init_poses, std::string map_file)
    {
        StatesMutex.lock();
        for(int i = 0; i < robot_names.size(); i++){
            States_Ptr->insert(std::pair<std::string, core::State>{robot_names[i], core::State{.X = std::get<0>(robot_init_poses[i]), .Y = std::get<1>(robot_init_poses[i]), .Dir = std::get<2>(robot_init_poses[i]), .VX = 0.0, .VY = 0.0, .W = 0.0}});
        }
    
        StatesMutex.unlock();
        setEnvironmentMap(map_file);


        for (auto &&pair : *States_Ptr)
        {
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_ptr = this->create_publisher<geometry_msgs::msg::Point>(pair.first + "/robot_pose", 10);
            robot_pose_ptr_map[pair.first] = pose_ptr;
        }
    }

    void CoreNode::setEnvironmentMap(std::string pic_name)
    {
        inverted_input_map = cv::imread(pic_name, cv::IMREAD_GRAYSCALE);
        if(!inverted_input_map.data)
        {
            std::cout<<"failed to read image from "<<pic_name<<std::endl;
        }
        else
        {   
            cv::flip(inverted_input_map, input_environment_map, 0);
            this->Map_Ptr->header.frame_id = "map";
            this->Map_Ptr->info.resolution = map_resolution;
            this->Map_Ptr->info.width = input_environment_map.cols;
            this->Map_Ptr->info.height = input_environment_map.rows;
            map_width = input_environment_map.cols;
            map_height = input_environment_map.rows;
            int data_size = this->Map_Ptr->info.width * this->Map_Ptr->info.height;
            this->Map_Ptr->data.resize(data_size, -1);
            for(int i = 0; i < data_size; i++)
            {
                int mat_value = (int)input_environment_map.data[i];
                // if(mat_value == 255) this->Map_Ptr->data[i] = 0;
                if(mat_value != 255) this->Map_Ptr->data[i] = 100;

            }
        }
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

            //publish robot pose message
            geometry_msgs::msg::Point g_pose;
            g_pose.x = state.X;
            g_pose.y = state.Y;
            g_pose.z = 0.0;

            robot_pose_ptr_map[pair.first]->publish(g_pose);

            ++id;
        }
        this->statesVisualizer_ptr->publish(poses);
        this->labelsVisualizer_ptr->publish(labels);
        this->StatesMutex.unlock_shared();
    }
} // namespace simulator::core
