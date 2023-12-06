# ME495 Final Project - Group 1 - Botrista

Authors: Stephen Ferro, Anuj Natraj, Carter DiOrio, Kyle Wang, and Jihai Zhao

Description: A collection of ROS2 packages to drive the Emika Franka robot arm to brew a cup of pour over coffee. 

Quickstart Instructions:
1. After building and sourcing the workspace, launch rviz and the required nodes using 'ros2 launch botrista botrista.launch.py'
2. To start the coffee-making routine, place a white cup in the cup hold in front of the robot. The robot will detect the cup and beign making coffee.

Nodes:
- camera_localizer: localizes the d435 and d45 cameras and publishes transforms for april tags seen by the cameras from the robot base
- coffee_grounds: controls the actions for picking up and dumping the coffee scoop
- cup_detection: handles detection of the coffee cup in the cup holder and triggers the rest of the routine. also publishes a transform to the top of the coffee cup
- delay_node: handles the delay service which is used to pause the robot for a specified time at certain points in the routine
- grasp_node: offers the grasp_process action, which is used to grap the "standard" handle used for the kettle, pot, and filter
- handle_detector: tracks the blue and green tape on the handles of the objects using the d405 camera and publishes a tf for the object handle
- kettle: handles action for picking up, pouring, and placing the kettle
- pick_filter: offers the action to pick up the coffee filter
- pot: handles action for picking up, pouring, and placing the coffee pot
- pouring: offers the pour action, which is used by the kettle to create spiral motions
- run_botrista: the main node which offers the make_coffee action
- tag_transform_lookup: 

Launchfiles:
- botrista.launch.py: 
- cup_detection.launch.py: 
- open_franka.launch.xml: 
- realsense.launch.py: 

Configuration instructions:
1. 