#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

def read_pose_from_file(filename):
    pose = {'position': {}, 'orientation': {}}
    with open(filename, 'r') as file:
        lines = file.readlines()
        pose['position']['x'] = float(lines[1].split(':')[1].strip())
        pose['position']['y'] = float(lines[2].split(':')[1].strip())
        pose['position']['z'] = float(lines[3].split(':')[1].strip())
        pose['orientation']['x'] = float(lines[5].split(':')[1].strip())
        pose['orientation']['y'] = float(lines[6].split(':')[1].strip())
        pose['orientation']['z'] = float(lines[7].split(':')[1].strip())
        pose['orientation']['w'] = float(lines[8].split(':')[1].strip())
    return pose

def move_to_pose(group, pose):
    # create target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = pose['position']['x']
    target_pose.position.y = pose['position']['y']
    target_pose.position.z = pose['position']['z']
    target_pose.orientation.x = pose['orientation']['x']
    target_pose.orientation.y = pose['orientation']['y']
    target_pose.orientation.z = pose['orientation']['z']
    target_pose.orientation.w = pose['orientation']['w']
    
    # set target pose
    group.set_pose_target(target_pose)
    
    # navigate
    plan = group.go(wait=True)
    
    # clean all
    group.stop()
    group.clear_pose_targets()

def main():
    # init ros node
    rospy.init_node('move_to_pose_node', anonymous=True)
    
    # init moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # init MoveGroupCommander
    group_name = "cr5_arm"  
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # get original pose from file
    original_pose = read_pose_from_file('needle_navigation/src/saved_pose_orignal.txt')
    
    # get final pose from file
    end_pose = read_pose_from_file('needle_navigation/src/saved_pose_end.txt')
    
    # first move to original pose
    rospy.loginfo("Moving to original pose...")
    move_to_pose(move_group, original_pose)
    
    # then move to end pose
    rospy.loginfo("Moving to end pose...")
    move_to_pose(move_group, end_pose)
    
    # shutdown
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()


