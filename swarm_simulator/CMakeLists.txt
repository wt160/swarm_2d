cmake_minimum_required(VERSION 3.5)
project(swarm_simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

aux_source_directory(core/src CORE_SRC)
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(${PROJECT_NAME}_core ${CORE_SRC})
ament_target_dependencies(${PROJECT_NAME}_core rclcpp nav_msgs geometry_msgs visualization_msgs OpenCV)



aux_source_directory(plugins/navigation NAVIGATION_PLUGIN_SRC)
add_library(${PROJECT_NAME}_navigation_plugin ${NAVIGATION_PLUGIN_SRC})
ament_target_dependencies(${PROJECT_NAME}_navigation_plugin rclcpp rclcpp_action nav_msgs swarm_interfaces geometry_msgs visualization_msgs)

aux_source_directory(plugins/slam SLAM_PLUGIN_SRC)
add_library(${PROJECT_NAME}_slam_plugin ${SLAM_PLUGIN_SRC})
ament_target_dependencies(${PROJECT_NAME}_slam_plugin rclcpp nav_msgs swarm_interfaces geometry_msgs visualization_msgs)

add_executable(core_test_node test/core_test_node.cpp)
target_link_libraries(core_test_node ${PROJECT_NAME}_core  ${PROJECT_NAME}_navigation_plugin ${PROJECT_NAME}_slam_plugin)
ament_target_dependencies(core_test_node rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(navigation_test_node test/navigation_test_node.cpp)
target_link_libraries(navigation_test_node ${PROJECT_NAME}_core  ${PROJECT_NAME}_navigation_plugin ${PROJECT_NAME}_slam_plugin)
ament_target_dependencies(navigation_test_node rclcpp nav_msgs geometry_msgs visualization_msgs swarm_interfaces)

add_executable(main_app app/main.cpp)
target_link_libraries(main_app ${PROJECT_NAME}_core  ${PROJECT_NAME}_navigation_plugin ${PROJECT_NAME}_slam_plugin)
ament_target_dependencies(main_app rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(experiment_recorder app/experiment_recorder.cpp)
target_link_libraries(experiment_recorder ${PROJECT_NAME}_core  ${PROJECT_NAME}_navigation_plugin ${PROJECT_NAME}_slam_plugin)
ament_target_dependencies(experiment_recorder rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(final_coverage_recorder app/final_coverage_recorder.cpp)
target_link_libraries(final_coverage_recorder ${PROJECT_NAME}_core  ${PROJECT_NAME}_navigation_plugin ${PROJECT_NAME}_slam_plugin)
ament_target_dependencies(final_coverage_recorder rclcpp nav_msgs geometry_msgs visualization_msgs)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    core_test_node
    navigation_test_node
    main_app
    experiment_recorder
    final_coverage_recorder
    DESTINATION lib/${PROJECT_NAME})

ament_package()
