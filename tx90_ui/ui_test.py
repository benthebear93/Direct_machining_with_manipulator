#!/usr/bin/env python2
import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
import openpyxl
import copy
import rospy
import pandas as pd
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import geometry_msgs
from math import pi
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from moveit_msgs.srv import GetPositionFK
import dill
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, rotation_matrix, concatenate_matrices,quaternion_matrix, euler_from_matrix

form_class = uic.loadUiType("form.ui")[0]

def all_close(goal, actual, tolerance):
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class WindowClass(QMainWindow, form_class) :
    def __init__(self) :
        super(WindowClass,self).__init__()
        self.setupUi(self)
        """
        SIGNAL
        """
        self.non_q = 0
        self.opt_q = 0
        self.read_jointq()  
        self.robot = self.StaubliScanning()
        self.non_posture1.clicked.connect(self.button1Function)
        self.non_posture2.clicked.connect(self.button2Function)
        self.non_posture3.clicked.connect(self.button3Function)
        self.non_posture4.clicked.connect(self.button4Function)
        self.non_posture5.clicked.connect(self.button5Function)
        self.non_posture6.clicked.connect(self.button6Function)
        self.non_posture7.clicked.connect(self.button7Function)
        self.non_posture8.clicked.connect(self.button8Function)
        self.non_posture9.clicked.connect(self.button9Function)
        self.non_posture10.clicked.connect(self.button10Function)
        self.non_posture11.clicked.connect(self.button11Function)
        self.non_posture12.clicked.connect(self.button12Function)
        self.non_posture13.clicked.connect(self.button13Function)
        self.non_posture14.clicked.connect(self.button14Function)
        self.non_posture15.clicked.connect(self.button15Function)
    
        self.opt_posture1.clicked.connect(self.Obutton1Function)
        self.opt_posture2.clicked.connect(self.Obutton2Function)
        self.opt_posture3.clicked.connect(self.Obutton3Function)
        self.opt_posture4.clicked.connect(self.Obutton4Function)
        self.opt_posture5.clicked.connect(self.Obutton5Function)
        self.opt_posture6.clicked.connect(self.Obutton6Function)
        self.opt_posture7.clicked.connect(self.Obutton7Function)
        self.opt_posture8.clicked.connect(self.Obutton8Function)
        self.opt_posture9.clicked.connect(self.Obutton9Function)
        self.opt_posture10.clicked.connect(self.Obutton10Function)
        self.opt_posture11.clicked.connect(self.Obutton11Function)
        self.opt_posture12.clicked.connect(self.Obutton12Function)
        self.opt_posture13.clicked.connect(self.Obutton13Function)
        self.opt_posture14.clicked.connect(self.Obutton14Function)
        self.opt_posture15.clicked.connect(self.Obutton15Function)

        self.reset.clicked.connect(self.resetFunction)


    def read_jointq(self):
        non_filename = 'ros_flat_v2.xlsx'
        opt_filename = 'test3.xlsx'
        df = pd.read_excel(non_filename, header=None, names=None, index_col=None)
        print("df", df)
        number_pose = len(df)
        self.non_q = []
        for i in range(1, number_pose):
            q = np.array(df.iloc[i][1:7])
            q = np.array([q[0],q[1],q[2],q[3],q[4],q[5]])
            q = np.deg2rad(q)
            self.non_q.append(q)

        df = pd.read_excel(opt_filename, header=None, names=None, index_col=None)
        number_pose = len(df)
        self.opt_q = []
        for i in range(1, number_pose):
            q = np.array(df.iloc[i][1:7])
            q = np.array([q[0],q[1],q[2],q[3],q[4],q[5]])
            q = np.deg2rad(q)
            self.opt_q.append(q)

    class StaubliScanning(object):
        def __init__(self):
            #super(StaubliScanning, self).__init__()
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node('move_group_python_interface', anonymous=True)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()

            group_name = "tx_90"
            self.move_group =  moveit_commander.MoveGroupCommander(group_name)
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
            planning_frame = self.move_group.get_planning_frame()
            eef_link = self.move_group.get_end_effector_link()
            group_names = self.robot.get_group_names()

        def go_to_joint_state(self, q):
            print("go to joint state", q)
            move_group = self.move_group

            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = q[0]
            joint_goal[1] = q[1]
            joint_goal[2] = q[2]
            joint_goal[3] = q[3]
            joint_goal[4] = q[4]
            joint_goal[5] = q[5]

            move_group.go(joint_goal, wait=True)

            move_group.stop()

            # For testing:
            current_joints = move_group.get_current_joint_values()
            # print("                              ")
            # print("current joint ",current_joints)
            # print("                              ")
            return all_close(joint_goal, current_joints, 0.01)

    def resetFunction(self):
        home_pose = np.deg2rad([0, 0, 90, 0, 0, 0])
        self.robot.go_to_joint_state(home_pose)
        print("moving home pose")

    def button1Function(self) :
        self.robot.go_to_joint_state(self.non_q[0])
        print("pose 1")

    def button2Function(self) :
        self.robot.go_to_joint_state(self.non_q[1])
        print("pose 2")
    
    def button3Function(self) :
        self.robot.go_to_joint_state(self.non_q[2])
        print("pose 3")
    
    def button4Function(self) :
        self.robot.go_to_joint_state(self.non_q[3])
        print("pose 4")

    def button5Function(self) :
        self.robot.go_to_joint_state(self.non_q[4])
        print("pose 5")

    def button6Function(self) :
        self.robot.go_to_joint_state(self.non_q[5])
        print("pose 6")

    def button7Function(self) :
        self.robot.go_to_joint_state(self.non_q[6])
        print("pose 7")

    def button8Function(self) :
        self.robot.go_to_joint_state(self.non_q[7])
        print("pose 8")

    def button9Function(self) :
        self.robot.go_to_joint_state(self.non_q[8])
        print("pose 9")

    def button10Function(self) :
        self.robot.go_to_joint_state(self.non_q[9])
        print("pose 10")

    def button11Function(self) :
        self.robot.go_to_joint_state(self.non_q[10])
        print("pose 11")

    def button12Function(self) :
        self.robot.go_to_joint_state(self.non_q[11])
        print("pose 12")

    def button13Function(self) :
        self.robot.go_to_joint_state(self.non_q[12])
        print("pose 13")

    def button14Function(self) :
        self.robot.go_to_joint_state(self.non_q[13])
        print("pose 14")

    def button15Function(self) :
        self.robot.go_to_joint_state(self.non_q[14])
        print("pose 15")
    # Non opt

    def Obutton1Function(self) :
        self.robot.go_to_joint_state(self.opt_q[0])
        print("pose 1")

    def Obutton2Function(self) :
        self.robot.go_to_joint_state(self.opt_q[1])
        print("pose 2")
    
    def Obutton3Function(self) :
        self.robot.go_to_joint_state(self.opt_q[2])
        print("pose 3")
    
    def Obutton4Function(self) :
        self.robot.go_to_joint_state(self.opt_q[3])
        print("pose 4")

    def Obutton5Function(self) :
        self.robot.go_to_joint_state(self.opt_q[4])
        print("pose 5")

    def Obutton6Function(self) :
        self.robot.go_to_joint_state(self.opt_q[5])
        print("pose 6")

    def Obutton7Function(self) :
        self.robot.go_to_joint_state(self.opt_q[6])
        print("pose 7")

    def Obutton8Function(self) :
        self.robot.go_to_joint_state(self.opt_q[7])
        print("pose 8")

    def Obutton9Function(self) :
        self.robot.go_to_joint_state(self.opt_q[8])
        print("pose 9")

    def Obutton10Function(self) :
        self.robot.go_to_joint_state(self.opt_q[9])
        print("pose 10")

    def Obutton11Function(self) :
        self.robot.go_to_joint_state(self.opt_q[10])
        print("pose 11")

    def Obutton12Function(self) :
        self.robot.go_to_joint_state(self.opt_q[11])
        print("pose 12")

    def Obutton13Function(self) :
        self.robot.go_to_joint_state(self.opt_q[12])
        print("pose 13")

    def Obutton14Function(self) :
        self.robot.go_to_joint_state(self.opt_q[13])
        print("pose 14")

    def Obutton15Function(self) :
        self.robot.go_to_joint_state(self.opt_q[14])
        print("pose 15")
    #Opt


if __name__ == "__main__" :
    # staubli_client = StaubliScanning()
    # staubli_client.go_to_joint_state()
    #QApplication :
    app = QApplication(sys.argv) 

    #WindowClass
    myWindow = WindowClass() 

    myWindow.show()

    app.exec_()
