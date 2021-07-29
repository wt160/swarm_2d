#include "fieldofview.hpp"

namespace simulator::plugin
{

    FieldOfView::FieldOfView(std::string robot_name, double simulate_frequency, int sight_length, std::shared_ptr<simulator::core::CoreNode>& core_ptr): rclcpp::Node{"field_of_view_" + robot_name}
    {
        core_ptr_ = core_ptr;
        robot_name_ = robot_name;

        //the shared_ptr to the shared map among all the robots 
        shared_map_ptr = core_ptr->Map_Ptr;
        //copy the original environment map from simulator core
        robot_map_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>(*(core_ptr->Map_Ptr));
        robot_map_ptr->header.frame_id = "map";
        pair<int, int> map_width_height = core_ptr_->getMapWidthAndHeight();
        map_width = map_width_height.first;
        map_height = map_width_height.second;

        field_of_view_frequency = simulate_frequency;
        field_of_view_ptr = this->create_wall_timer(std::chrono::microseconds((int)(1000000 / field_of_view_frequency)),
                                                            std::bind(&FieldOfView::simulateSLAM, this));
        line_of_sight_length = sight_length;
        vector<pair<int, int>> circle;

        center_to_circle_set.clear();

        double degree_increment = 360.0 / (2 * 3.14159 * line_of_sight_length) - 0.6;
        std::cout<<"degree increment:"<<degree_increment<<std::endl;
        for(double j = 0.0; j < 360.0; j+= 0.3){
            double rad = j / 180.0 * 3.14159;
            double xmove = cos(rad);
            double ymove = sin(rad);
            double dist = 0;
            vector<pair<int, int>> line_list;
            double x = 0.0; double y = 0.0;
            line_list.push_back(make_pair(0, 0));
            while(dist < line_of_sight_length){
                x += xmove;
                y += ymove;
                line_list.push_back(make_pair(round(x), round(y)));
                // std::cout<<"add ("<<round(x)<<","<<round(y)<<")"<<std::endl;
                dist += 1;
            }
            if(check_line_list.find(line_list) == check_line_list.end()){
                check_line_list.insert(line_list);
            }
        }

        std::cout<<"line_list size:"<<check_line_list.size()<<std::endl;
        robot_map_pub_ptr = this->create_publisher<nav_msgs::msg::OccupancyGrid>(robot_name_ + "/robot_map", 10);
        initMapWithUnknownMask();
    }


    void FieldOfView::initMapWithUnknownMask()
    {
        if(robot_map_ptr->data.size() > 0)
        {
            for(int i = 0; i < robot_map_ptr->data.size(); i++)
            {
                if(robot_map_ptr->data[i] == 0)
                {
                    robot_map_ptr->data[i] = -1;
                }
            }

        }
    }

    bool FieldOfView::isValidMapPoint(int coord_x, int coord_y)
    {
        if(coord_x >= 0 && coord_x < map_width && coord_y >= 0 && coord_y < map_height)
        {
            return true;
        }
        else{
            return false;
        }
    }

    void FieldOfView::simulateSLAM()
    {
        // std::cout<<robot_name_<<" slam"<<std::endl;
        double map_resolution = core_ptr_->getMapResolution();
    
        // peer_pose_in_peer_frame_dict_ = peer_pose_in_peer_frame_dict;
        //get current robot's current pose from simulator::core
        simulator::core::State curr_state = (*(core_ptr_->States_Ptr))[robot_name_];
        double curr_x = curr_state.X;
        double curr_y = curr_state.Y;
        double curr_theta = curr_state.Dir;

        int curr_x_coord = (round)(curr_x /map_resolution); 
        int curr_y_coord = (round)(curr_y /map_resolution); 
        // std::cout<<robot_name_<<" x:"<<curr_x_coord<<" y:"<<curr_y_coord<<std::endl;
        pair<int, int> robot_pos = make_pair(curr_x_coord, curr_y_coord);
        for(auto i = check_line_list.begin(); i != check_line_list.end(); i++)
        {
            pair<int, int> check_line_endpoint = i->back();
            int end_check_x = curr_x_coord + check_line_endpoint.first;
            int end_check_y = curr_y_coord + check_line_endpoint.second;

            for(int j = 0; j < i->size(); j++){
                int check_x = curr_x_coord + (*i)[j].first;
                int check_y = curr_y_coord + (*i)[j].second;
                if(check_x >= 0 && check_x < map_width && check_y >= 0 && check_y < map_height)
                {
                    int shared_check_data = shared_map_ptr->data[check_y * map_width + check_x];
                    // std::cout<<shared_check_data<<std::endl;
                    if(shared_check_data == -1)
                    {
                        shared_map_ptr->data[check_y * map_width + check_x] = 0;
                    }
                    else if(shared_check_data == 100)
                    {
                        break;
                    }

                }

            }


            // if(isValidMapPoint(end_check_x, end_check_y))
            // {
            //     int end_data_index = end_check_y * map_width + end_check_x;
            //     if(robot_map_ptr->data[end_data_index] == 0)
            //     {
            //         //the end of this line of sight is explored, we can safely 
            //         //skip the check of this line of sight
            //         continue;
            //     }
            // }

            // int check_x = 0;
            // int check_y = 0;
            // int check_index = 0;
            // int higher_bound = check_line_list[i].size();
            // int lower_bound = 0;
            // do
            // {
            //     check_index = round((higher_bound + lower_bound) / 2);
            //     check_x = curr_x_coord + check_line_list[i][check_index].first;
            //     check_y = curr_y_coord + check_line_list[i][check_index].second;
            //     if(isValidMapPoint(check_x, check_y))
            //     {
            //         int check_data = robot_map_ptr->data[check_y * map_width + check_x];
            //         if(check_data == -1)
            //         {
                        
            //             for(int k = lower_bound; k <= check_index; k++){
            //                 int temp_check_x = curr_x_coord + check_line_list[i][k].first;
            //                 int temp_check_y = curr_y_coord + check_line_list[i][k].second;

            //                 robot_map_ptr->data[temp_check_y * map_width + temp_check_x] = 0;
            //             }

            //             lower_bound = check_index;
            //         }
            //         else if(check_data == 100)
            //         {
            //             higher_bound = check_index;
            //         }
            //         else if(check_data == 0)
            //         {
            //             lower_bound = check_index;
            //         }
            //     }
            //     else
            //     {
            //         higher_bound = check_index;
            //     }
            // }
            // while(lower_bound < higher_bound - 1);


            // std::cout<<robot_name_<<" after"<<std::endl;



            for(int j = 0; j < i->size(); j++){
                int check_x = curr_x_coord + (*i)[j].first;
                int check_y = curr_y_coord + (*i)[j].second;
                if(check_x >= 0 && check_x < map_width && check_y >= 0 && check_y < map_height)
                {
                    int check_data = robot_map_ptr->data[check_y * map_width + check_x];
                    // if(check_line_list[i][j].first == 6 && check_line_list[i][j].second == 6){
                    //     std::cout<<"(6,6)"<<check_data<<std::endl;
                    // }
                    if(check_data == -1)
                    {
                        robot_map_ptr->data[check_y * map_width + check_x] = 0;
                    }
                    else if(check_data == 100)
                    {
                        break;
                    }

                }
                else{
                    break;
                }

            }
            
        }
        robot_map_pub_ptr->publish(*robot_map_ptr);
    }

    vector<pair<int, int>> FieldOfView::getLineOfSight(int monster_x, int monster_y)
    {
        int t, x, y, abs_delta_x, abs_delta_y, sign_x, sign_y, delta_x, delta_y;
        vector<pair<int, int>> los;
    /* Delta x is the players x minus the monsters x    *
        * d is my dungeon structure and px is the players  *
        * x position. monster_x is the monsters x position passed *
        * to the function.                                 */
        delta_x =  - monster_x;

        /* delta_y is the same as delta_x using the y coordinates */
        delta_y =  - monster_y;

        /* abs_delta_x & abs_delta_y: these are the absolute values of delta_x & delta_y */
        abs_delta_x = abs(delta_x);
        abs_delta_y = abs(delta_y);

        /* sign_x & sign_y: these are the signs of delta_x & delta_y */
        sign_x = SGN(delta_x);
        sign_y = SGN(delta_y);

        /* x & y: these are the monster's x & y coords */
        x = monster_x;
        y = monster_y;
        los.push_back(make_pair(x, y));
        /* The following if statement checks to see if the line *
            * is x dominate or y dominate and loops accordingly    */
        if(abs_delta_x > abs_delta_y)
        {
            /* X dominate loop */
            /* t = twice the absolute of y minus the absolute of x*/
            t = abs_delta_y * 2 - abs_delta_x;
            do
            {
                if(t >= 0)
                {
                    /* if t is greater than or equal to zero then *
                    * add the sign of delta_y to y                    *
                    * subtract twice the absolute of delta_x from t   */
                    y += sign_y;
                    t -= abs_delta_x*2;
                }
                
                /* add the sign of delta_x to x      *
                * add twice the adsolute of delta_y to t  */
                x += sign_x;
                t += abs_delta_y * 2;
                
                los.push_back(make_pair(x, y));
                /* check to see if we are at the player's position */
            /* keep looping until the monster's sight is blocked *
            * by an object at the updated x,y coord             */
            }
            while(x != 0 || y != 0);
        
            /* NOTE: sight_blocked is a function that returns true      *
            * if an object at the x,y coord. would block the monster's *
            * sight                                                    */

            /* the loop was exited because the monster's sight was blocked *
            * return FALSE: the monster cannot see the player             */
            std::reverse(los.begin(), los.end());
            return los;
        }
        else
        {
            /* Y dominate loop, this loop is basically the same as the x loop */
            t = abs_delta_x * 2 - abs_delta_y;
            do
            {
                if(t >= 0)
                {
                    x += sign_x;
                    t -= abs_delta_y * 2;
                }
                y += sign_y;
                t += abs_delta_x * 2;
                
                los.push_back(make_pair(x, y));

            }
            while(x != 0 || y != 0);
            std::reverse(los.begin(), los.end());
            return los;
        }
    }

}