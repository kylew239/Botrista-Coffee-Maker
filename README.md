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

Launchfiles:
- botrista.launch.py: the main launchfile to launch botrista. It launches all the nodes listed above except camera_localizer, as well as running the two launch files below. Once launched, the robot is ready to execture the make_coffee action once it detects a cup.
- open_franka.launch.xml: Launches moveit and RViz for the Franka arm.
- realsense.launch.py: Launches both the d435i and the d405, the april tag node, and the camera_localizer node.

'make_coffee' Routine High Level Overview:
1. The cup_detection node wait until it detects an empty cup placed in the detection area, then publishes a message on the 'coffee_start' topic, which cuases the run_botrista node to start the routine
2. Picks up the Filter from filter stand using the pick_filter action
3. Places the Filter on the Pot using the place_filter_in_pot action
4. Pick up a full scoop of coffee gournds and dump them in filter. This is all handled by the scoop action
5. Pick up Kettle with the pick_kettle action
6. Pour water from kettle using the pour_action action
7. Place Kettle back on kettle stand w ith the place_kettle action
8. Wait for coffee grounds to soak using the delay service
9. Pick up Filter from the Pot using the pick_filter_in_pot action
10. Place Filter on filter stand (place_filter action)
11. Pick up Pot from pot stand (pick_pot action)
12. Pour Coffee (pour_pot action)
13. Put the Pot back on pot stand (place_pot action)
