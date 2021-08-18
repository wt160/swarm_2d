# swarm_2d
To test the Astar navigation, run the commands below, 

ros2 run swarm_simulator main_app

ros2 run swarm_simulator navigation_test_node target_x  target_y

ros2 run rviz2 rviz2 

#use the swarm.rviz under src folder


# Published topics:
1, /robot_name/map   

type: nav_msgs/msg/OccupancyGrid

publish robot local map at 5Hz

2, /robot_name/robot_pose

type: geometry_msgs/msg/Point

publish robot pose in global frame at 30Hz

3, /map

type: nav_msgs/msg/OccupancyGrid

publish robot group shared map at 30Hz
