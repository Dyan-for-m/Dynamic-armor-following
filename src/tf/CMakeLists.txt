cmake_minimum_required(VERSION 3.8)
project(tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(static_transform_tf2_broadcaster src/static_transform_tf2_broadcaster.cpp)
ament_target_dependencies(
   static_transform_tf2_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

install(TARGETS
   static_transform_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
   
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



# 现在我已经完成了装甲板的识别，可以通过订阅主题/detector/marker [visualization_msgs/msg/MarkerArray]（检测到有新的位置信息会及时更新） 来得到相对相机的装甲板的xyz和四元数，同时我也实现了tutlebot3的base_link
# 和camera_link的tf静态变换。
# 现在情况如下：
# 1.相机的xyz定义和ros2里面的定义不相同，两者的关系是（设相机的坐标是x1,y1,z1，ros2标准坐标是x2,y2,z2）：x1=-y2,y1=-z2,z1=x2，四元数的xyzw可能也不同，请你看情况推理。
# 2.我需要利用现有官方已给出的现有的navigation2 的 一个action来实现机器人跟随装甲板的功能，具体来说是我拿着装甲板移动，不只是发一次goal，而是经常更新目标点，此时机器人可以持续追踪运动中的装甲板并且运动跟随，我现在需要一个client来调用这个action，需要你来帮我完成。
# 3.给我写这个client的具体操作，从建立ros2的功能包开始，到Cmakelist.txt和package.xml的编写，到具体的代码编写，我需要你来帮我完成这个功能包的建立。
# 4.给我在终端的命令输入，来实现功能的调用。
# 5.告诉我怎么添加滤波和稳定：对检测的位姿进行时间上的滤波，如卡尔曼滤波，提高定位稳定性。