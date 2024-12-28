cmake_minimum_required(VERSION 3.8)
project(my_armor_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# 如果需要OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(armor_follower_node src/armor_follower_node.cpp)
ament_target_dependencies(armor_follower_node 
  rclcpp rclcpp_action geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs nav2_msgs std_msgs nav_msgs)

# 如果用到OpenCV
target_link_libraries(armor_follower_node ${OpenCV_LIBS})

install(TARGETS armor_follower_node
  DESTINATION lib/${PROJECT_NAME})

  add_executable(armor_follower_client_node src/armor_follower_client_node.cpp)
ament_target_dependencies(armor_follower_client_node 
  rclcpp rclcpp_action geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs nav2_msgs std_msgs nav_msgs)

# 如果用到OpenCV
target_link_libraries(armor_follower_client_node ${OpenCV_LIBS})

install(TARGETS armor_follower_client_node
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

#这个client还是有点问题，我现在突然发现虽然/deterctor/marker的frame_id是camera_color_optical_frame，是相机的坐标系，但是他的父级是camera_color_frame,是标准的ros2坐标系，
#具体的tf结构关系为base_link->camera_link->camera_color_frame_>camera_color_optical_frame，同时再改进一下，用