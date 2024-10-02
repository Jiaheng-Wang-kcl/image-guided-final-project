This is my image-guided-final-project repositry.

`cd <your_workspace>`

`catkin_make`

`source <Your workspace>/devel/setup.bash`


Install 3D Slicer with Needle_path_Planning module and OpenIGTLink module

Install ROS2, ROS-IGTL-Bridge, MoveIt package, RVIZ

For 3D Slicer:

    Import Needle_path_Planning module

    Choose the desire data

    set the max cortex angle and needle length threshold

    press "Count the number of entry points and target points"

    press "Plan the optimal path"

    press "Prepare data to send to ROS"

    Then use OpenIGTLink to send the entrypoint and target point to ROS2


For ROS2 side

`roslaunch moveit_needle_sim demo.launch`

`ros2 run needle_navigation pose`

`ros2 run needle_navigation move_pose`

