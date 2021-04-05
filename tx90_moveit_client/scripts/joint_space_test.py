#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def senario_pos_dic_key_init():
    pos_dic = {
    'init_pose':[1.0, 0.0, 0.0, 0.0, 0.05, 0.05 ,1.428],
    'cfrp_center':[0.5, -0.5, 0.5, -0.5, 0.745, 0.0, 0.2],
    'cfrp_pos1':[0.5, -0.5, 0.5, -0.5, 0.695, -0.05, 0.2],
    'cfrp_pos2':[0.5, -0.5, 0.5, -0.5, 0.695, 0.05, 0.2],
    'cfrp_pos3':[0.5, -0.5, 0.5, -0.5, 0.795, -0.05, 0.2],
    'cfrp_pos4':[0.5, -0.5, 0.5, -0.5, 0.795, 0.05, 0.2],
    }
    senario_dic ={
    'senario1':[0, pi/4, pi/4, pi/2, 0, pi/5],
    'senario2':[0, pi/5, pi/3, pi/7, pi/3, -pi/5],
    'senario3':[0, pi/4, pi/3, -pi/7, pi/3, pi/5],
    'senario4':[0, pi/6, pi/4, pi/7, -pi/6, pi/5]
    }
    return pos_dic, senario_dic

def pose_target_gen(pos):
    # Target Position generate 
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = pos[0]
    pose_target.orientation.x = pos[1]
    pose_target.orientation.y = pos[2]
    pose_target.orientation.z = pos[3]
    pose_target.position.x = pos[4]
    pose_target.position.y = pos[5]
    pose_target.position.z = pos[6]
    return pose_target

def move_to_pos(move_group, posdic_key):
    pose_target = pose_target_gen(posdic_key)
    move_group.set_pose_target(pose_target)
    plan1 = move_group.plan()

    move_group.go(wait=True)
    move_group.stop()

def save_jointval(move_group, senario_num):
    current_joints = move_group.get_current_joint_values()
    print senario_num, ":", current_joints
    # f = open(senario_num + ".txt", 'w')
    # data_start = senario_num + "first senario\n"
    # f.write(data_start)
    # data = " ".join(str(x) for x in current_joints)
    # f.write(data)
    # f.close()


def senario(move_group, var_pose_key):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = var_pose_key[0]
    joint_goal[1] = var_pose_key[1]
    joint_goal[2] = var_pose_key[2]
    joint_goal[3] = var_pose_key[3]
    joint_goal[4] = var_pose_key[4]
    joint_goal[5] = var_pose_key[5]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

def four_point_test(move_group, senario_num, pos_dic, senario_dic):
    if senario_num == 1:
        senario(move_group, senario_dic['senario1'])
        move_to_pos(move_group, pos_dic['cfrp_pos1'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1)
        senario(move_group, senario_dic['senario1'])
        move_to_pos(move_group,pos_dic['cfrp_pos2']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1)
        senario(move_group, senario_dic['senario1'])
        move_to_pos(move_group,pos_dic['cfrp_pos3'])
        save_jointval(move_group, senario_num) 
        rospy.sleep(0.1)
        senario(move_group, senario_dic['senario1'])
        move_to_pos(move_group, pos_dic['cfrp_pos4'])
        save_jointval(move_group, senario_num) 
        rospy.sleep(0.1)
        move_to_pos(move_group, pos_dic['init_pose'])

    elif senario_num == 2:
        senario(move_group, senario_dic['senario2'])
        move_to_pos(move_group, pos_dic['cfrp_pos1'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1)  
        senario(move_group, senario_dic['senario2'])
        move_to_pos(move_group, pos_dic['cfrp_pos2'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1)  
        senario(move_group, senario_dic['senario2'])
        move_to_pos(move_group, pos_dic['cfrp_pos3']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        senario(move_group, senario_dic['senario2'])
        move_to_pos(move_group, pos_dic['cfrp_pos4'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        move_to_pos(move_group, pos_dic['init_pose'])

    elif senario_num == 3:
        senario(move_group, senario_dic['senario3'])
        move_to_pos(move_group, pos_dic['cfrp_pos1']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        senario(move_group, senario_dic['senario3'])
        move_to_pos(move_group, pos_dic['cfrp_pos2']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        senario(move_group, senario_dic['senario3'])
        move_to_pos(move_group, pos_dic['cfrp_pos3']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        senario(move_group, senario_dic['senario3'])
        move_to_pos(move_group, pos_dic['cfrp_pos4'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        move_to_pos(move_group, pos_dic['init_pose'])

    elif senario_num == 4:
        senario(move_group, senario_dic['senario4'])
        move_to_pos(move_group, pos_dic['cfrp_pos1'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1)  
        senario(move_group, senario_dic['senario4'])
        move_to_pos(move_group, pos_dic['cfrp_pos2']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        senario(move_group, senario_dic['senario4'])
        move_to_pos(move_group, pos_dic['cfrp_pos3']) 
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        senario(move_group, senario_dic['senario4'])
        move_to_pos(move_group, pos_dic['cfrp_pos4'])
        save_jointval(move_group, senario_num)
        rospy.sleep(0.1) 
        move_to_pos(move_group, pos_dic['init_pose'])

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('staubli_control', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)

    # Get group_commander from MoveGroupCommander
    group_name = "tx_90"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    '''display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)'''

    #position and senario position dictionray initiate
    pos_dic, senario_dic = senario_pos_dic_key_init()   
    #initial default pose
    senario(move_group, pos_dic['init_pose'])
    rospy.sleep(1) 
    test = [0, 1.5708, -1.5708, 0, 0, 0]
    #senario(move_group, senario_dic['senario4'])
    # test = [pi/3, pi/3, pi/3, pi/3, pi/3, pi/3]
    senario(move_group, test)
    quit()

if __name__ == "__main__":
    main()