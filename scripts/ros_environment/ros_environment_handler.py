#! /usr/bin/env python

import os

import rospy

import tf

import math

from time import *


from nav_msgs.msg import Path as path_msg_type
from nav_msgs.msg import Odometry as odom_msg_type
from nav_msgs.msg import OccupancyGrid as occgrid_msg_type
from map_msgs.msg import OccupancyGridUpdate as occgrid_update_msg_type
from geometry_msgs.msg import PoseWithCovariance as PSWC_msg_type
from geometry_msgs.msg import PoseWithCovarianceStamped as PSWC_stamped_msg_type
from geometry_msgs.msg import Pose as pose_msg_type
from geometry_msgs.msg import Twist as twist_msg_type
from geometry_msgs.msg import PoseStamped as pose_stamped_msg_type
from rosgraph_msgs.msg import Clock as clock_msg_type
from sensor_msgs.msg import Joy as joy_msg_type
from sensor_msgs.msg import LaserScan as laserscann_msg_type

#real robot msgs
from trajectory_msgs.msg import JointTrajectory as joint_trajectory_msg_type
from sensor_msgs.msg import JointState as joint_state_msg_type
from trajectory_msgs.msg import JointTrajectoryPoint as joint_trajectory_point_msg_type

#real robot move base world reset
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped

from std_srvs.srv import Empty

import numpy as np


class ros_environment(object):

    def callback_path(self, new_path):

        print("New Path received!")
        self.path = new_path
        self.path_available = True


    def callback_js(self, joint_state):
        #print("new joint states")

        self.is_omega_left_wheel = joint_state.velocity[0]
        self.is_omega_right_wheel = joint_state.velocity[1]


    def callback_odom(self, odom_msg):
        #print("Odom")

        if self.resetting == False and self.init == True:

            #print("Odom callback")
            # if simulation mode is active calc wheel vels from odom
            if self.simulation:
                self.is_omega_left_wheel = (odom_msg.twist.twist.linear.x + (odom_msg.twist.twist.angular.z * self.axis_length)/2)*2/self.wheel_diameter
                self.is_omega_right_wheel = -(odom_msg.twist.twist.linear.x - (odom_msg.twist.twist.angular.z * self.axis_length)/2)*2/self.wheel_diameter

                if(math.fabs(self.is_omega_left_wheel) < 0.02):
                    self.is_omega_left_wheel = 0.0

                if (math.fabs(self.is_omega_right_wheel) < 0.02):
                    self.is_omega_right_wheel = 0.0

                #print("Drehzahl links: " + str(self.is_omega_left_wheel) + " Drehzahl rechts: " + str(self.is_omega_right_wheel))


            #distance data is also needed
            self.robot_current_odom_pose_stamped.header = odom_msg.header
            self.robot_current_odom_pose_stamped.pose = odom_msg.pose.pose

            # print("Position in X in /odom frame: " + str(robot_current_odom_pose_stamped.pose.position.x))
            # transform geometry_msgs/PoseStamped in /odom Frame to /map Frame
            # wait for transformation
            self.transform_available = False
            self.transform_timeout_cnt = 0
            while self.transform_available == False and self.error == False:
                try:
                    #(trans, rot) = self.tflistener.lookupTransform('odom', 'map', rospy.Time(0))
                    self.robot_current_odom_pose_stamped.header.stamp = self.tflistener.getLatestCommonTime("/map",
                                                                                                            self.robot_current_odom_pose_stamped.header.frame_id)
                    self.transform_available = True
                    # transform goal from map frame to base_link frame
                    self.robot_current_map_pose_stamped = self.tflistener.transformPose('map', self.robot_current_odom_pose_stamped)
                    self.error = False
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    # print("failed!")
                    self.transform_timeout_cnt = self.transform_timeout_cnt + 1
                    if self.transform_timeout_cnt > 100:
                        #print("Transformation from odom to map was not available all the time")
                        #self.error = True
                        self.error = False
                    continue

            #update robot state space goal position
            # wait for transformation
            self.transform_available = False
            self.transform_timeout_cnt = 0

            #update timestamp for transformation with tf
            self.goal_pose_stamped_map_frame.header.stamp = rospy.get_rostime()

            while self.transform_available == False and self.error == False:
                try:
                    #(trans, rot) = self.tflistener.lookupTransform('map', 'base_link', rospy.Time(0))
                    #print(trans)
                    self.goal_pose_stamped_map_frame.header.stamp = self.tflistener.getLatestCommonTime("/base_link",
                                                                                                        self.goal_pose_stamped_map_frame.header.frame_id)
                    #self.transform_available = True
                    # transform goal from map frame to base_link frame
                    self.goal_pose_stamped_base_link_frame = self.tflistener.transformPose('base_link',
                                                                                      self.goal_pose_stamped_map_frame)
                    self.transform_available = True
                    #print(self.goal_pose_stamped_base_link_frame)
                    self.error = False
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as tf_exception:
                    print(tf_exception)
                    #raise
                    self.transform_timeout_cnt = self. transform_timeout_cnt + 1
                    if self.transform_timeout_cnt > 100:
                        #print("Transformation was not available all the time")
                        #self.error = True
                        self.error = False
                    continue

            #store goal pose base_link frame in robot state space
            #print("GOAL X: " + str(self.goal_pose_stamped_base_link_frame.pose.position.x))
            #print("GOAL Y: " + str(self.goal_pose_stamped_base_link_frame.pose.position.y))

            self.goal_position_linear_x = self.goal_pose_stamped_base_link_frame.pose.position.x
            self.goal_position_linear_y = self.goal_pose_stamped_base_link_frame.pose.position.y
            self.goal_position_linear_z = self.goal_pose_stamped_base_link_frame.pose.position.z

            self.goal_position_angular_x = self.goal_pose_stamped_base_link_frame.pose.orientation.x
            self.goal_position_angular_y = self.goal_pose_stamped_base_link_frame.pose.orientation.y
            self.goal_position_angular_z = self.goal_pose_stamped_base_link_frame.pose.orientation.z
            self.goal_position_angular_w = self.goal_pose_stamped_base_link_frame.pose.orientation.w

            try:
                #get relative rotation from robot to goal frame
                quadGoal = (
                    self.goal_pose_stamped_map_frame.pose.orientation.x,
                    self.goal_pose_stamped_map_frame.pose.orientation.y,
                    self.goal_pose_stamped_map_frame.pose.orientation.z,
                    self.goal_pose_stamped_map_frame.pose.orientation.w)
                quadRobot = (
                    self.robot_current_map_pose_stamped.pose.orientation.x,
                    self.robot_current_map_pose_stamped.pose.orientation.y,
                    self.robot_current_map_pose_stamped.pose.orientation.z,
                    self.robot_current_map_pose_stamped.pose.orientation.w)

                quatDiff = tf.transformations.quaternion_multiply(quadGoal, tf.transformations.quaternion_inverse(quadRobot))

                euler = tf.transformations.euler_from_quaternion(quatDiff)

                self.yaw_diff = (euler[2]*180/math.pi)
                #print("yaw = ", (euler[2]*180/math.pi))
            finally:
                # close connection, remove subcsriptions, etc
                #print("YAW diff calc failed!")
                a = 0

        self.new_odom_received = True




    def __init__(self, simulation, state_dim, wheel_diameter, axis_length, additional_weight):

        print("ROS ENVIRONMENT HANDLER V0.4")


        self.init = False
        self.simulation = simulation
        self.resetting = False

        self.path = path_msg_type()
        self.path_available = False
        self.new_goal = False

        self.normalize_data = True

        #ROS
        self.tflistener = tf.TransformListener(True, rospy.Duration(10.0))

        #ROS Topic Subscriber
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", path_msg_type, self.callback_path)
        self.odom_sub = rospy.Subscriber("/odom", odom_msg_type, self.callback_odom)

        #ROS Topic Publisher
        self.pub_goal_pose = rospy.Publisher('/goal_pose', pose_stamped_msg_type, queue_size=10)

        if not self.simulation:
            print("Using a real Robot")
            #topics for wheel control
            self.sub_joint_state = rospy.Subscriber("/drives/joint_state", joint_state_msg_type, self.callback_js)
            self.pub_set_vel = rospy.Publisher('/drives/joint_trajectory', joint_trajectory_msg_type, queue_size=10)
        else:
            self.pub_cmd_vel = rospy.Publisher('/cmd_vel', twist_msg_type, queue_size=10)



        #Robot State Space
        self.state_dim = state_dim
        self.state_as_nparray = np.zeros([state_dim])

        self.is_omega_left_wheel = 0.0
        self.is_omega_right_wheel = 0.0

        self.yaw_diff = 0.0

        self.new_goal = False

        # tmp storage for goal pose base_link frame
        self.goal_position_linear_x = 0.0
        self.goal_position_linear_y = 0.0
        self.goal_position_linear_z = 0.0

        self.goal_position_angular_x = 0.0
        self.goal_position_angular_y = 0.0
        self.goal_position_angular_z = 0.0
        self.goal_position_angular_w = 1.0

        self.additional_weight = additional_weight

        self.scale_action_factor = 1.0

        #Robot information
        self.wheel_diameter = wheel_diameter
        self.axis_length = axis_length

        #Goal pose that has to be reached
        self.goal_pose_stamped_base_link_frame = pose_stamped_msg_type()

        #init goal pose in map frame
        self.goal_pose_stamped_map_frame = pose_stamped_msg_type()
        self.goal_pose_stamped_map_frame.header.stamp = rospy.get_rostime()
        self.goal_pose_stamped_map_frame.header.frame_id = "map"
        # position
        self.goal_pose_stamped_map_frame.pose.position.x = 0.0
        self.goal_pose_stamped_map_frame.pose.position.y = 0.0
        self.goal_pose_stamped_map_frame.pose.position.z = 0.0
        # orientation as quaternion
        self.goal_pose_stamped_map_frame.pose.orientation.x = 0.0
        self.goal_pose_stamped_map_frame.pose.orientation.y = 0.0
        self.goal_pose_stamped_map_frame.pose.orientation.z = 0.0
        self.goal_pose_stamped_map_frame.pose.orientation.w = 1.0

        #other poses for distance calculation
        self.robot_current_odom_pose_stamped = pose_stamped_msg_type()
        self.robot_current_map_pose_stamped = pose_stamped_msg_type()

        #real robot interaction
        #self.current_jt = joint_trajectory_msg_type()
        #self.current_js = joint_state_msg_type()

        #initial pose for reset
        #header
        self.robot_initial_pose = PSWC_stamped_msg_type()
        self.robot_initial_pose.header.stamp = rospy.get_rostime()
        self.robot_initial_pose.header.frame_id = "map"
        #position
        self.robot_initial_pose.pose.pose.position.x = 0.0
        self.robot_initial_pose.pose.pose.position.y = 0.0
        self.robot_initial_pose.pose.pose.position.z = 0.0
        #orientation as quaternion
        self.robot_initial_pose.pose.pose.orientation.x = 0.0
        self.robot_initial_pose.pose.pose.orientation.y = 0.0
        self.robot_initial_pose.pose.pose.orientation.z = 0.0
        self.robot_initial_pose.pose.pose.orientation.w = 1.0
        #covariance
        #float64 [36]
        self.robot_initial_pose.pose.covariance = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1)

        #Flag for new data via odom topic handled
        self.new_odom_received = False


        self.error = False
        self.init = True

    def get_error(self):
        return self.error

    def is_state_space_usable(self):
        return self.new_odom_received

    def get_state(self):
        #this method returns the current state of the mobile robot

        state_as_nparray = np.zeros([self.state_dim])

        # normalize from [-6, 6] to [-1, 1]
        state_as_nparray[0] = self.is_omega_left_wheel/self.scale_action_factor
        state_as_nparray[1] = self.is_omega_right_wheel/self.scale_action_factor

        #print "State Vel Left : " + str(self.is_omega_left_wheel) + "State Vel Right : " + str(
            #self.is_omega_right_wheel)
        #print "Scaled State Vel Left : " + str(state_as_nparray[0]) + "Scaled State Vel Right : " + str(
            #state_as_nparray[1])


        #normalize from [-3, 3] to [-1, 1]
        if self.goal_position_linear_x > 3:
            self.goal_position_linear_x = 3
        if self.goal_position_linear_x < -3:
            self.goal_position_linear_x = -3
        if self.goal_position_linear_y > 3:
            self.goal_position_linear_y = 3
        if self.goal_position_linear_y < -3:
            self.goal_position_linear_y = -3

        state_as_nparray[2] = self.goal_position_linear_x/3
        state_as_nparray[3] = self.goal_position_linear_y/3

        state_as_nparray[4] = self.yaw_diff/180

        print "Goal X: " + str(state_as_nparray[2])
        print "Goal Y: " + str(state_as_nparray[3])
        print "Goal Alpha: " + str(state_as_nparray[4])


        #print("GET STATE: X: " + str(self.robot_current_map_pose_stamped.pose.position.x) + " Y: " + str(self.robot_current_map_pose_stamped.pose.position.y))

        #mark state space values as used
        self.new_odom_received = False

        return [state_as_nparray, self.robot_current_map_pose_stamped]

    def execute_action(self, action):
        # this method executes an action depended on real robot or simulation

        if self.simulation == True:
            #transform wheel rotation to velocity command because simulation can not handle wheel rotation commands

            print "Action Left : " + str(action[0]) + "Action Right : " + str(action[1])

            # scale noise action to real action space
            self.target_omega_left_wheel = action[0] * self.scale_action_factor
            self.target_omega_right_wheel = action[1] * self.scale_action_factor

            print "Scaled Action Left : " + str(self.target_omega_left_wheel) + "Scaled Action Right : " + str(self.target_omega_right_wheel)

            #publish cmd_vel to ROS
            self.cmd_vel_msg = twist_msg_type()

            # forward kinematics for differential drive
            self.cmd_vel_msg.linear.x = 0.5 * (self.target_omega_left_wheel -  self.target_omega_right_wheel) * self.wheel_diameter * 0.5
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0

            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = (((self.target_omega_left_wheel +  self.target_omega_right_wheel) * self.wheel_diameter )/2) /self.axis_length

            #print("VelX = " + str(self.cmd_vel_msg.linear.x) + " VelZ = " +str(self.cmd_vel_msg.angular.z))

            self.pub_cmd_vel.publish(self.cmd_vel_msg)

            self.pub_goal_pose.publish(self.goal_pose_stamped_map_frame)

        else:

            ##################################################
            # This is for real robot interaction             #
            ##################################################

            current_jt = joint_trajectory_msg_type()
            current_jt.header.stamp = rospy.get_rostime()
            #set each data for 4 motors because neo_relayboard only handles 4 or 8 motors
            current_jt.joint_names.append("wheel_front_left")
            current_jt.joint_names.append("wheel_front_right")
            current_jt.joint_names.append("unused")
            current_jt.joint_names.append("unused")

            # if action[0] > 0 and action[1] > 0:
            #     action[0] = action[0] + (-1)
            #     action[1] = action[1] + (-1)
            #
            # if action[0] < 0 and action[1] < 0:
            #     action[0] = action[0] + (-1)
            #     action[1] = action[1] + (-1)

            #add point with target velocities
            current_point_left = joint_trajectory_point_msg_type()
            current_point_left.velocities.append(action[0]*self.scale_action_factor*-1)
            current_point_left.velocities.append(action[1]*self.scale_action_factor*-1)
            current_point_left.velocities.append(0.0)
            current_point_left.velocities.append(0.0)
            current_jt.points.append(current_point_left)

            #publish drive command
            self.pub_set_vel.publish(current_jt)

            self.pub_goal_pose.publish(self.goal_pose_stamped_map_frame)

        return 1

    def set_goal_pose(self, goal_pose):
        print("set Goal pose in map frame")

        self.goal_pose_stamped_map_frame.header.stamp = rospy.get_rostime()
        self.goal_pose_stamped_map_frame.header.frame_id = "map"
        self.goal_pose_stamped_map_frame.pose.position.x = goal_pose.position.x
        self.goal_pose_stamped_map_frame.pose.position.y = goal_pose.position.y
        self.goal_pose_stamped_map_frame.pose.position.z = goal_pose.position.z
        self.goal_pose_stamped_map_frame.pose.orientation.x = goal_pose.orientation.x
        self.goal_pose_stamped_map_frame.pose.orientation.y = goal_pose.orientation.y
        self.goal_pose_stamped_map_frame.pose.orientation.z = goal_pose.orientation.z
        self.goal_pose_stamped_map_frame.pose.orientation.w = goal_pose.orientation.w

        self.new_goal = True

        # mark state space values as used
        self.new_odom_received = False

    def is_path_available(self):
        return self.path_available
        self.new_goal = True

    def get_path(self):
        self.path_available = False
        return self.path
        # mark state space values as used
        self.new_odom_received = False

    def reset_error(self):
        self.error = False

