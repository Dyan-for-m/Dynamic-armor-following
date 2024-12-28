# Dynamic-armor-following

The armor plate here is the armor plate used in the RM competition, and navigation2 is used in the code to achieve the tracking and navigation of the armor plate.

Environment: Ubuntu22.04 ROS2 Humble, using the physical robot Turtlebot3 Burger to operate, where the camera uses realsense D435i.

## Installation and Usage

1. Make sure [Navigation2 can be used](https://docs.nav2.org/getting_started/index.html#installation).

2. create a workspace and create a src file, clone the package in the src file.

3. build the workspace.
cd to your workspace and run the following command:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=release
```

4. source the workspace

```bash
source install/setup.bash
```

5. Start the robot's chassis motor and the installed camera

copy the following command in the terminal of robot:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

and turn on the camera.
6. run the node and navigation

cd to your workspace,
copy the following command in your terminal:

```bash
ros2 launch tf track.launch.py
ros2 run my_armor_follower armor_follower_node
```

7. You can observe whether the armor plate is detected by rviz, visualize it by subscribing/detector/marker, if it is not detected, you can try to adjust the exposure parameters of the camera

## Note 

If it is different from my device, please make the following changes: 

1. You need to change the name of your subscription according to the topic of your camera, that is, you need to modify the name of the subscription topic  `cam_info_sub_` & `img_sub_`from `src/armor_detector/src/detector_node.cpp`.

2. Observe your tf and the tf of the camera, you can view it with the code: `ros2 run rqt_tf_tree rqt_tf_tree`, modify the tf connection of `src/tf/src/static_transform_tf2_broadcaster.cpp`.

3. Modify the name of the camera topic in the `/src/turtle_ws/src/my_armor_follower/src/armor_follower_node.cpp`. (you can check the official documentation or check the TF tree of the camera by yourself) to convert the camera coordinate system to the ROS2 coordinate system.

4. Please adjust the parameters of `src/tf/params/nav2_params.yaml` according to the attributes of your robot.

