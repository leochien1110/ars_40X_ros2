cmake_minimum_required(VERSION 3.5)
project(ars_40x)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find ros2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(socket_can REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
# find third party dependencies
find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
include_directories(${socket_can_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(ars_40x_msg
    src/cluster_list.cpp
    src/motion_input_signals.cpp
    src/object_list.cpp
    src/radar_cfg.cpp
    src/radar_state.cpp
    )

add_library(ars_40x_can
    src/ars_40x_can.cpp
    )
target_link_libraries(ars_40x_can ars_40x_msg ${socket_can_LIBRARIES})

add_executable(ars_40x_ros
    src/ros/ars_40x_ros.cpp
    src/ros/cluster_list_ros.cpp
    src/ros/motion_input_signals_ros.cpp
    src/ros/object_list_ros.cpp
    src/ros/radar_cfg_ros.cpp
    src/ros/radar_state_ros.cpp
    )
target_link_libraries(ars_40x_ros ars_40x_can ${socket_can_LIBRARIES})
ament_target_dependencies(ars_40x_ros
    costmap_converter
    costmap_converter_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    std_msgs
    std_srvs
    visualization_msgs
    perception_msgs
    )

add_executable(ars_40x_rviz
    src/ros/ars_40x_rviz.cpp
    )
target_link_libraries(ars_40x_rviz ${PCL_LIBRARIES})
ament_target_dependencies(ars_40x_rviz
    rclcpp
    costmap_converter
    costmap_converter_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    std_srvs
    visualization_msgs
    perception_msgs
)

add_executable(ars_40x_obstacle_array
    src/ros/ars_40x_obstacle_array.cpp
    )
ament_target_dependencies(ars_40x_obstacle_array
    rclcpp
    costmap_converter
    costmap_converter_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    std_srvs
    visualization_msgs
    perception_msgs
    )

# test
add_executable(cluster_pub
    test/cluster_pub.cpp
    )
ament_target_dependencies(cluster_pub
    rclcpp
    geometry_msgs
    perception_msgs
)

add_executable(service_test
    test/service_test.cpp
    )
ament_target_dependencies(service_test
    rclcpp
    perception_msgs
)

# install
install(
    TARGETS
    # node
    ars_40x_can
    ars_40x_msg
    ars_40x_ros
    ars_40x_rviz
    ars_40x_obstacle_array
    # test
    cluster_pub
    service_test
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# launch and config files
install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
