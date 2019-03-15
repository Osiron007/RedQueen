#Basic
import numpy as np
import json
import math

#ROS for Python
import rospy

#Tensorflow
import tensorflow as tf

#Keras
from keras import backend as K

from model.ActorNetwork import ActorNetwork

####################
import sys

from time import *

import ConfigParser

import numpy as np

import random

import argparse

#####################ROS#########################################
import rospy

from ros_environment import ros_environment_handler as REH

from geometry_msgs.msg import Pose as pose_msg_type
from geometry_msgs.msg import PoseStamped as pose_stamped_msg_type


#####################LEARNING#########################################
#from reward_calculator import reward_calculation as RC

def ST_WAITFORPATH():


    #print("Waiting for a Path to follow")

    if ros_handler.is_path_available():

        path = ros_handler.get_path()
        print "poses in path: " + str(len(path.poses))
        for pose in path.poses:
            goal_point_list.append([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        return "ST_STARTUP"
    else:
        return "ST_WAITFORPATH"

def ST_STARTUP():
    #print("ST_STARTUP")
    global startup_cnt

    global step

    global ros_handler

    global goal_pose

    global start_time_episode

    global episode

    global episode_done

    global s_t

    global goal_point_list

    global goal_point_index

    global new_goal_wait

    if startup_cnt == 0:
        print("Startup Episode")
        startup_cnt = startup_cnt + 1
        return "ST_STARTUP"
    if startup_cnt == 5:
        # set initial pose
        #ros_handler.set_initial_pose()
        # get rnd goal pose

	if len(goal_point_list) >= 1:
            point = goal_point_list[goal_point_index]
            goal_point_index = goal_point_index + 1

            goal_pose.position.x = point[0]
            goal_pose.position.y = point[1]
            goal_pose.position.z = point[2]
            goal_pose.orientation.x = point[3]
            goal_pose.orientation.y = point[4]
            goal_pose.orientation.z = point[5]
            goal_pose.orientation.w = point[6]

            # set goal pose
            ros_handler.set_goal_pose(goal_pose)

            startup_cnt = startup_cnt + 1
            new_goal_wait = 0
            return "ST_STARTUP"
    if startup_cnt > 10:

        #reset variables
        startup_cnt = 0
        start_time_episode = time()
        total_reward = 0.0
        step = 0
        episode = episode + 1
        episode_done = False

        #get starting state space
        [s_t, tmp] = ros_handler.get_state()

        print("Startup done")
        return "ST_EXECUTING"

    startup_cnt = startup_cnt + 1
    return "ST_STARTUP"


def ST_EXECUTING():
    #print("ST_EXECUTING")
    #episode handling
    global step
    global episode
    global episode_done

    global ros_handler

    #endless drive
    global new_goal_wait
    global goal_point_list
    global goal_point_index

    #goal pose
    global goal_pose
    # robot pose
    global robot_current_map_pose_stamped

    #state and action space
    global state_dim
    global action_dim

    global s_t


    #Visualization and Logging
    global visu_show_step_data
    global logging_save_transition_for_debug
    global logging_save_reward


    a_t = np.zeros([1, action_dim])

    #predict actions
    a_t = actor.model.predict(s_t.reshape(1, s_t.shape[0])) * 3

    #execute actions
    ros_handler.execute_action(a_t[0])

    #get state
    [s_t1, robot_current_map_pose_stamped] = ros_handler.get_state()


    # check if robot is in goal area
    if math.fabs(s_t1[2]) < 0.6 and math.fabs(s_t1[3]) < 0.6:
        #print("goal area reached -> end episode")
        #episode_done = True
        if new_goal_wait >= 0:
            new_goal_wait = 0
            #endless drive
            #set next goal on the fly
            print("Index: " + str(goal_point_index))
            if(goal_point_index > len(goal_point_list)-1):
                goal_point_index = len(goal_point_list)-1

            point = goal_point_list[goal_point_index]
            goal_point_index = goal_point_index + 1

            goal_pose.position.x = point[0]
            goal_pose.position.y = point[1]
            goal_pose.position.z = point[2]
            goal_pose.orientation.x = point[3]
            goal_pose.orientation.y = point[4]
            goal_pose.orientation.z = point[5]
            goal_pose.orientation.w = point[6]

            # set goal pose
            ros_handler.set_goal_pose(goal_pose)

            print "Dist to Goal: X=" + str(s_t1[2]) + " Y=" + str(s_t1[3])

            if goal_point_index > len(goal_point_list)-1:

                #goal_point_index = 0
                if(math.fabs(s_t1[2]) < 0.03 and math.fabs(s_t1[3]) < 0.03 and math.fabs((s_t1[4]*180)) < 3):
                    a_t[0][0] = 0.0
                    a_t[0][1] = 0.0
                    ros_handler.execute_action(a_t[0])
                    print("Goal reached")
                    end_time_episode = time()

                    time_elapsed = end_time_episode - start_time_episode

                    print("Start time: " + str(start_time_episode))
                    print("End time: " + str(end_time_episode))
                    print("Elapsed: " + str(time_elapsed))
                    return "ST_WAITFORPATH"

        else:
            new_goal_wait = new_goal_wait + 1

        if ros_handler.is_path_available():
            return "ST_WAITFORPATH"


    if logging_save_transition_for_debug:
        print("Save transition for debug")
        print("Left Wheel: " + str(s_t[0]) + " ---> " + str(s_t1[0]))
        print("Right Wheel: " + str(s_t[1]) + " ---> " + str(s_t1[1]))

        print("Goal X pos: " + str(s_t[2]) + " ---> " + str(s_t1[2]))
        print("Goal Y pos: " + str(s_t[3]) + " ---> " + str(s_t1[3]))
        print("Goal Angle Diff: " + str(s_t[4]*180) + " ---> " + str(s_t1[4]*180))

    states_tmp = np.zeros(state_dim)
    states_tmp[:] = s_t1[:]
    s_t[:] = states_tmp[:]

    if visu_show_step_data:
        print("Episode", episode, "Step", step, "Action", a_t)#, "Reward", r_t)
    step = step + 1

    return "ST_EXECUTING"



def ST_RESET_WORLD():
    #print("ST_RESET_WORLD")

    global reset_world_cnt

    global ros_handler

    if reset_world_cnt == 0:
        print("Reset World")
        ros_handler.reset_world()
    if reset_world_cnt > 5:
        reset_world_cnt = 0
        return "ST_STARTUP"

    reset_world_cnt = reset_world_cnt + 1
    return "ST_RESET_WORLD"

#----------------------------------------------------------------------------->
#------------------------errorhandler----------------------------------------->
#----------------------------------------------------------------------------->
# Error handling
def errorhandler():
    #close this program
    print("--------------------------Critical Error-------------------------")
    #exit script
    sys.exit()



###############################################################################
#############################  MAIN  ##########################################
###############################################################################
if __name__ == '__main__':


    # print("#############Read configuration###################")
    config = ConfigParser.ConfigParser()
    config.readfp(open('configuration.cfg'))

    # hyperparameter
    global state_dim
    global action_dim
    print("#############Hyperparameter#####################")
    state_dim = config.getint("hyperparameter", "state_dim")
    print("State dimension: " + str(state_dim))
    action_dim = config.getint("hyperparameter", "action_dim")
    print("Action dimension: " + str(action_dim))

    # robot parameter
    print("##############Robot parameter#####################")
    wheel_diameter = config.getfloat("robot", "wheel_diameter")
    print("wheel_diameter: " + str(wheel_diameter))
    axis_length = config.getfloat("robot", "axis_length")
    print("axis_length: " + str(axis_length))

    # logging and visualization
    global visu_show_cycle_time
    global visu_show_step_data
    global logging_save_transition_for_debug
    print("###########Logging and Visualization#################")
    visu_show_cycle_time = config.getboolean("visualization", "visu_show_cycle_time")
    print("visu_show_cycle_time: " + str(visu_show_cycle_time))
    visu_show_step_data = config.getboolean("visualization", "visu_show_step_data")
    print("visu_show_step_data: " + str(visu_show_step_data))
    logging_save_transition_for_debug = config.getboolean("logging", "logging_save_transition_for_debug")
    print("logging_save_transition_for_debug: " + str(logging_save_transition_for_debug))
    print("\n")


    USE_TARGET_ACTOR = False
    STATE_SIZE = 5
    ACTION_SIZE = 2
    # NOT NEEDED HERE
    BATCH_SIZE = 32
    TAU = 0.0
    LRA = 0.0
    ################

    sess = tf.Session()
    K.set_session(sess)

    global actor
    actor = ActorNetwork(sess, STATE_SIZE, ACTION_SIZE, BATCH_SIZE, TAU, LRA)

    global graph
    graph = tf.get_default_graph()

    # Now load the weight
    print("Now we load the weight")
    try:
        actor.model.load_weights("model/actormodel.h5")
        actor.target_model.load_weights("model/actormodel.h5")
        print("Weight load successfully")
    except:
        print("\033[91m ########################################## \033[0m")
        print("\033[91m Cannot find the weight \033[0m")
        print("\033[91m ########################################## \033[0m")
        exit()

    stateAction = {
        "ST_STARTUP": ST_STARTUP,
        "ST_RESET_WORLD": ST_RESET_WORLD,
        "ST_EXECUTING": ST_EXECUTING,
        "ST_WAITFORPATH": ST_WAITFORPATH,
    }

    # initial state
    activeState = "ST_WAITFORPATH"

    rospy.init_node('redqueen', anonymous=False)

    rate = rospy.Rate(10) #Hz

    global ros_handler
    ros_handler = REH.ros_environment(simulation=True, state_dim=state_dim ,wheel_diameter=wheel_diameter, axis_length=axis_length, additional_weight=0.0)

    global episode_done
    episode_done = False

    global startup_cnt
    startup_cnt = 0

    global reset_world_cnt
    reset_world_cnt = 0

    global episode
    episode = 0

    global step
    step = 0

    global s_t
    s_t = np.zeros([state_dim])

    global goal_pose
    goal_pose = pose_msg_type()

    global robot_current_map_pose_stamped
    robot_current_map_pose_stamped = pose_stamped_msg_type()

    global goal_point_list
    goal_point_list = []

    global goal_point_index
    goal_point_index = 1

    try:
        while not rospy.is_shutdown():
            #time.sleep(0.1)
            s = time()
            oldState = activeState
            activeState = stateAction.get(activeState, errorhandler)()

            if visu_show_cycle_time:
                s1 = time()
                diff = s1 - s
                print("Cycle Time: " + str(diff*1000) + " [ms]")

            rate.sleep()
    finally:
        # close connection, remove subcsriptions, etc
        print("ERROR CATCHED! -> Exit")




