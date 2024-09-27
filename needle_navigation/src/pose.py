#!/usr/bin/env python
import sys 
import rospy
import moveit_commander
import geometry_msgs.msg

def save_current_pose():
    # init ros node
    rospy.init_node('save_pose_node', anonymous=True)
    
    # init moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # init robot
    robot = moveit_commander.RobotCommander()
    
    # init MoveGroupCommander
    group_name = "cr5_arm"  
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # record current pose
    current_pose = move_group.get_current_pose().pose

    # save current pose to files
    with open("saved_pose.txt", "w") as file:
        file.write("Position:\n")
        file.write(f"x: {current_pose.position.x}\n")
        file.write(f"y: {current_pose.position.y}\n")
        file.write(f"z: {current_pose.position.z}\n")
        file.write("Orientation:\n")
        file.write(f"x: {current_pose.orientation.x}\n")
        file.write(f"y: {current_pose.orientation.y}\n")
        file.write(f"z: {current_pose.orientation.z}\n")
        file.write(f"w: {current_pose.orientation.w}\n")

    rospy.loginfo("Current pose saved to saved_pose.txt")

if __name__ == "__main__":
    try:
        save_current_pose()
    except rospy.ROSInterruptException:
        pass


