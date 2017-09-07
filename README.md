# Yirsoon_environment_mapping
This is part of the project I'm doing in order to generate a map from an indoor planar environment, which is at the moment, 3rd floor of my university. I've followed ROS tutorials and used part of the motors' code from my_personal_robotic_companion

The order of commands used for mapping are the following:

T1 roscore

T2 roslaunch kinect_rosserial.launch

T3 roslauch odom_nav.launch

T4 roslaunch gmapping.launch

T5 rosrun teleop_twist_keyboard teleop_twist_keyboard.py

T6 rosrun rviz rviz
