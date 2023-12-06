# ME495 Final Project - Group 1 - Botrista

## Authors: 
Stephen Ferro, Anuj Natraj, Carter DiOrio, Kyle Wang, and Jihai Zhao

## Description: 
A collection of ROS2 packages to drive the Emika Franka robot arm to brew a cup of pour over coffee. 

## Quickstart Instructions:
1. Use 'git clone git@github.com:ME495-EmbeddedSystems/final-project-CarterDiOrio.git' to clone the repositiory into the 'src' directory of a new ROS2 workspace.
2. Build and source the workspace.
3. After building and sourcing the workspace, launch rviz and the required nodes using 'ros2 launch botrista botrista.launch.py'.
4. To start the coffee-making routine, place a white cup in the cup hold in front of the robot. The robot will detect the cup and beign making coffee.

## Nodes:
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

## Launchfiles:
- botrista.launch.py: the main launchfile to launch botrista. It launches all the nodes listed above except camera_localizer, as well as running the two launch files below. Once launched, the robot is ready to execture the make_coffee action once it detects a cup.
- open_franka.launch.xml: Launches moveit and RViz for the Franka arm.
- realsense.launch.py: Launches both the d435i and the d405, the april tag node, and the camera_localizer node.

## 'make_coffee' Routine High Level Overview:
- This system is essentially a series of service and action calls which cause the robot to brew coffee when called in order.
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

## Lessons Learned and Future Work:
- The biggest issues we encoutered with this project was moveit getting the robot into configurations that it could not get itself out of. This happened most frequently with motions that had a lot of path or goal constraints, such as moving the scoop to the pot or pouring the pot into the cup. We were able to work around these issues largely by tuning the constraints and by inserting more known intermediate positions for the robot to go to before reaching the goal pose. One idea to further improve this issue would be to have moveit find multiple paths to the goal, then choose the best option, rather than just picking the first solution it finds with RRT. Perhaps it would also be possible to write code allow moveit to take into account future commands as well as the current command. This could allow it to plan a path from which it does not get stuck. 
- Another issue we encountered was the weight of the kettle. Though we were somewhat close to the Franka's payload limit, we expected that it should have been able to carry more water than it was able too. If we over filled the kettle, the robot would sometimes fail to pick up the kettle because it thaught it had collieded with something. Perhaps we could slightly increase payload by further tuning the Franka payload variables.
- Additionally, there are some additional action the robot could complete the would make for a more full-featured demo. We wanted the robot to be able to scoop it's own coffee grounds, turn the kettle on and boil water by itself, and throw away old coffee grounds. With these features, the demo would essentially reset itself after each run. 

## Video:
[![CLICK HERE FOR VIDEO](//[https://i.ytimg.com/an_webp/s52ro0K72f4/mqdefault_6s.webp?du=3000&sqp=CLKxv6sG&rs=AOn4CLC6vXOBmmBzgHAVLhHjjB4Zef74Tw](https://i.ytimg.com/an_webp/s52ro0K72f4/mqdefault_6s.webp?du=3000&sqp=CLKxv6sG&rs=AOn4CLC6vXOBmmBzgHAVLhHjjB4Zef74Tw))](//www.youtube.com/watch?v=s52ro0K72f4)
