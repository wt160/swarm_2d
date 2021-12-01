# 启动与初始化

swarm仿真器可以由如下命令启动：

```shell
ros2 run swarm_simulator main_app --ros-args --params-file xxx

```

其中`xxx`是用来指定仿真器初始化采用的地图、机器人数量与坐标以及其他各项参数的文件，通过ros的参数系统传递入程序。一个参数文件的例子如下：

```yaml
simulator_mainapp:
  ros__parameters:
    map: /root/swarm_simulator_ws/src/swarm_2d/swarm_simulator/app/warehouse_01.png #必须指定绝对坐标
    names: [robot01, robot02]
    xs: [2.0, 5.0]
    ys: [2.0, 1.0]
    zs: [0.0, 0.0]
```

> 由于ros2的参数系统不支持复杂类型数组，因此只能将机器人的名称、坐标各项参数分别放入不同数组中。

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

4, /slam_map

type: nav_msgs/msg/OccupancyGrid

publish shared map with the slam characteristics

# 测试工具
1, final_coverage_recorder用来测试一次试验中，所有机器人的重合程度，需要提供机器人列表和用于存储结果的文件，结果会添加在结果文件末尾。
在所有探索完成之后，依赖地图topic： /map, robot01/map, ...

使用样例：

ros2 run swarm_simulator final_coverage_recorder ~/test.txt --ros-args -p "robot_list:=[robot01, robot02]"

2, experiment_recorder用于记录一次试验中覆盖面积随时间变化的数据
依赖地图topic: /map
使用样例：
ros2 run swarm_simulator experiment_recorder ~/test.txt

