cmake_minimum_required(VERSION 3.5)
project(delta_lidar)

set(delta_lidar_SDK_PATH "./sdk/")

FILE(GLOB delta_lidar_SDK_SRC 
 "${delta_lidar_SDK_PATH}/src/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
 ${delta_lidar_SDK_PATH}/include
 ${delta_lidar_SDK_PATH}/src
 ${catkin_INCLUDE_DIRS}
)


add_executable(delta_lidar_node src/node.cpp ${delta_lidar_SDK_SRC})
target_link_libraries(delta_lidar_node ${catkin_LIBRARIES} -lrt)

add_executable(delta_lidar_node_client src/client.cpp)
target_link_libraries(delta_lidar_node_client ${catkin_LIBRARIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs)
ament_target_dependencies(delta_lidar_node_client
    "rclcpp"
    "sensor_msgs"
    )
ament_target_dependencies(delta_lidar_node
    "rclcpp"
    "sensor_msgs"
    )
ament_package()

install(TARGETS delta_lidar_node delta_lidar_node_client
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz sdk
 DESTINATION share/${PROJECT_NAME}
 USE_SOURCE_PERMISSIONS
)


