#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class arm_main:

    # Constructor
    def __init__(self):

        rospy.init_node('set_joint_angles_node', anonymous=True)

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_max_velocity_scaling_factor(1)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles, group):
        self._planning_group = group
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_max_velocity_scaling_factor(1)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        # flag_plan = self._group.go(wait=False)

        # list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        # pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class aRmMoveit Deleted." + '\033[0m')


def main():

    hand = arm_main()
    arm = [
            math.radians(18),
            math.radians(-21),
            math.radians(137),
            math.radians(-31),
            math.radians(33),
            math.radians(-50),  
            ]


    index_joint_group = [
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0),
    ]


    middle_joint_group = [
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0),
    ]


    ring_joint_group = [
            math.radians(-14),
            math.radians(80),
            math.radians(80),
            math.radians(30),
    ]

    pinky_joint_group = [
            math.radians(-12),
            math.radians(86),
            math.radians(81),
            math.radians(6),
    ]

    thumb_joint_group = [
            math.radians(51),
            math.radians(30),
            math.radians(-48),
            math.radians(-45),
            math.radians(-37),
    ]


    # while not rospy.is_shutdown():
    hand.set_joint_angles(arm, group="arm_group")
    rospy.sleep(0.5)
    hand.set_joint_angles(index_joint_group, group="index_group")
    rospy.sleep(0.5)
    hand.set_joint_angles(middle_joint_group, group="middle_group")
    rospy.sleep(0.5)
    hand.set_joint_angles(ring_joint_group, group="ring_group")
    rospy.sleep(0.5)
    hand.set_joint_angles(pinky_joint_group, group="pinky_group")
    rospy.sleep(0.5)
    hand.set_joint_angles(thumb_joint_group, group="thumb_group")
    del hand

if __name__ == '__main__':
    main()