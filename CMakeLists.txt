cmake_minimum_required(VERSION 3.8)
project(bt_topic_trigger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(cnr_param REQUIRED)

# Build the skill executer node
add_executable(bt_topic_trigger_node
    src/bt_topic_trigger_node.cpp
    )
target_link_libraries(bt_topic_trigger_node cnr_param::cnr_param)
ament_target_dependencies(bt_topic_trigger_node rclcpp std_msgs behaviortree_ros2 btcpp_ros2_interfaces)

# =======
# INSTALL
# =======
install(TARGETS
    bt_topic_trigger_node
    DESTINATION lib/${PROJECT_NAME}
    )

#install(DIRECTORY launch config trees
#    DESTINATION share/${PROJECT_NAME})


ament_package()
