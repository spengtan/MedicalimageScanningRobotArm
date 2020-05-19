#!/usr/bin/env python
# -*- coding: UTF-8 -*-

""" vrep interface script that connects to vrep, reads and sets data to objects through vrep remote API  """

import time
import numpy as np
import vrep
import config
import math

# V-REP data transmission modes:
WAIT = vrep.simx_opmode_oneshot_wait
ONESHOT = vrep.simx_opmode_oneshot
STREAMING = vrep.simx_opmode_streaming
BUFFER = vrep.simx_opmode_buffer
BLOCKING = vrep.simx_opmode_blocking

robotID = -1
robotEndID = -1
#peopleID = -1
novelID = -1
kinect_depthID = -1
#left_motorID = -1
#right_motorID = -1
joint1ID = -1
joint2ID = -1
joint3ID = -1
joint4ID = -1
joint5ID = -1
joint6ID = -1
collisionID = -1
clientID = -1

rag2deg = 180.0 / np.pi

#pos_absolute = np.full(3, -1, dtype = np.float64)  # Robot pose in the world coordination: x(m), y(m), theta(rad)
#pos_relative = np.full(3, -1, dtype = np.float64)  # People pose relative to robot: x(m), y(m), theta(rad)
pos_absolute = np.full(6, -1, dtype = np.float64)  # Robot pose in the world coordination: x(m), y(m), theta(rad)


def show_msg(message):
    """ send a message for printing in V-REP """
    vrep.simxAddStatusbarMessage(clientID, message, WAIT)
    return


def connect():
    """ Connect to the simulator"""
    ip = '127.0.0.1'
    port = 19999
    vrep.simxFinish(-1)  # just in case, close all opened connections
    global clientID
    clientID = vrep.simxStart(ip, port, True, True, 3000, 5)
    # Connect to V-REP
    if clientID == -1:
        import sys
        sys.exit('\nV-REP remote API server connection failed (' + ip + ':' +
                 str(port) + '). Is V-REP running?')
    print('Connected to Remote API Server')  # show in the terminal
    show_msg('Python: Hello')    # show in the VREP
    time.sleep(0.5)
    return


def disconnect():
    """ Disconnect from the simulator"""
    # Make sure that the last command sent has arrived
    vrep.simxGetPingTime(clientID)
    show_msg('ROBOT: Bye')
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
    time.sleep(0.5)
    return


# def start():
#     """ Start the simulation (force stop and setup)"""
#     stop()
#     setup_devices()
#     vrep.simxStartSimulation(clientID, ONESHOT)
#     time.sleep(0.5)
#     # Solve a rare bug in the simulator by repeating:
#     setup_devices()
#     vrep.simxStartSimulation(clientID, ONESHOT)
#     time.sleep(0.5)
#     return


def stop():
    """ Stop the simulation """
    vrep.simxStopSimulation(clientID, ONESHOT)
    time.sleep(0.5)

def pause():
    """ pause the simulation """
    vrep.simxPauseSimulation(clientID, ONESHOT)
    time.sleep(0.5)


def start():
    setup_devices()
    # vrep.simxSynchronous(clientID, True)
    vrep.simxStartSimulation(clientID, ONESHOT)
    time.sleep(0.5)
    return

def setup_devices():
    """ Assign the devices from the simulator to specific IDs """
    # global robotID, left_motorID, right_motorID, peopleID, kinect_depthID, collisionID
    global robotID, robotEndID, joint1ID, joint2ID, joint3ID, joint4ID, joint5ID, joint6ID, novelID
    # res: result (1(OK), -1(error), 0(not called))
    # robot
    #res, robotID = vrep.simxGetObjectHandle(clientID, 'robot#', WAIT)
    # people
    #res, peopleID = vrep.simxGetObjectHandle(clientID, 'Bill_collisionShapeForPathPlanning#0', WAIT)
    # motors
    #res, left_motorID = vrep.simxGetObjectHandle(clientID, 'leftMotor#', WAIT)
    #res, right_motorID = vrep.simxGetObjectHandle(clientID, 'rightMotor#', WAIT)
    res, robotID = vrep.simxGetObjectHandle(clientID, 'UR3#', WAIT)
    res, robotEndID = vrep.simxGetObjectHandle(clientID, 'UR3_connection#', WAIT)
    res, joint1ID = vrep.simxGetObjectHandle(clientID, 'UR3_joint1#', WAIT)
    res, joint2ID = vrep.simxGetObjectHandle(clientID, 'UR3_joint2#', WAIT)
    res, joint3ID = vrep.simxGetObjectHandle(clientID, 'UR3_joint3#', WAIT)
    res, joint4ID = vrep.simxGetObjectHandle(clientID, 'UR3_joint4#', WAIT)
    res, joint5ID = vrep.simxGetObjectHandle(clientID, 'UR3_joint5#', WAIT)
    res, joint6ID = vrep.simxGetObjectHandle(clientID, 'UR3_joint6#', WAIT)

    res, novelID = vrep.simxGetObjectHandle(clientID, 'novel#', WAIT)


    # res, pos_joints1 = vrep.simxGetObjectPosition(clientID, joint1ID, -1, STREAMING)
    # res, pos_joints2 = vrep.simxGetObjectPosition(clientID, joint2ID, -1, STREAMING)
    # res, pos_joints3 = vrep.simxGetObjectPosition(clientID, joint3ID, -1, STREAMING)
    # res, pos_joints4 = vrep.simxGetObjectPosition(clientID, joint4ID, -1, STREAMING)
    # res, pos_joints5 = vrep.simxGetObjectPosition(clientID, joint5ID, -1, STREAMING)
    # res, pos_joints6 = vrep.simxGetObjectPosition(clientID, joint6ID, -1, STREAMING)



    # kinetic
    # res, kinect_depthID = vrep.simxGetObjectHandle(clientID, 'kinect_depth#', WAIT)
    # if res == vrep.simx_return_ok:  # [debug]
    #    print("vrep.simxGetDistanceHandle executed fine")
    # collision object
    # res, collisionID = vrep.simxGetCollisionHandle(clientID, "Collision#", BLOCKING)

    # start up devices
    # wheels
    # vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
    # vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)

    # pose
    # vrep.simxGetObjectPosition(clientID, robotID, -1, MODE_INI)
    # vrep.simxGetObjectOrientation(clientID, robotID, -1, MODE_INI)
    # vrep.simxGetObjectPosition(clientID, peopleID, robotID, MODE_INI)
    # vrep.simxGetObjectOrientation(clientID, peopleID, robotID, MODE_INI)

    res, ang_joint1 = vrep.simxGetJointPosition(clientID, joint1ID, STREAMING)
    res, ang_joint2 = vrep.simxGetJointPosition(clientID, joint2ID, STREAMING)
    res, ang_joint3 = vrep.simxGetJointPosition(clientID, joint3ID, STREAMING)
    res, ang_joint4 = vrep.simxGetJointPosition(clientID, joint4ID, STREAMING)
    res, ang_joint5 = vrep.simxGetJointPosition(clientID, joint5ID, STREAMING)
    res, ang_joint6 = vrep.simxGetJointPosition(clientID, joint6ID, STREAMING)

    # vrep.simxSetJointTargetPosition(clientID, joint1ID, 0, MODE_INI)
    # vrep.simxSetJointTargetPosition(clientID, joint2ID, 0, MODE_INI)
    # vrep.simxSetJointTargetPosition(clientID, joint3ID, 0, MODE_INI)
    # vrep.simxSetJointTargetPosition(clientID, joint4ID, 0, MODE_INI)
    # vrep.simxSetJointTargetPosition(clientID, joint5ID, 0, MODE_INI)
    # vrep.simxSetJointTargetPosition(clientID, joint6ID, 0, MODE_INI)
    #

    # res, pos_robot_end = vrep.simxGetObjectPosition(clientID, robotEndID, -1, STREAMING)
    # res, ori_robot_end = vrep.simxGetObjectOrientation(clientID, robotEndID, -1, STREAMING)
    #
    # res, pos_novel = vrep.simxGetObjectPosition(clientID, novelID, -1, STREAMING)
    # res, ori_novel = vrep.simxGetObjectOrientation(clientID, novelID, -1, STREAMING)


    # collision
    # vrep.simxReadCollision(clientID, collisionID, STREAMING)
    # kinect
    # vrep.simxGetVisionSensorDepthBuffer(clientID, kinect_depthID, STREAMING)
    return


def fetch_kinect():
    res, resolution, depth = vrep.simxGetVisionSensorDepthBuffer(clientID, kinect_depthID, BUFFER)
    depthData = np.array(depth)
    return depthData

# def get_robot_pose2d():
#     """ return the pose of the robot relative to world coordination:  [ x(m), y(m), Theta(rad) ] """
#     global pos_absolute
#     res, pos = vrep.simxGetObjectPosition(clientID, robotID, -1, MODE)
#     res, ori = vrep.simxGetObjectOrientation(clientID, robotID, -1, MODE)
#     pos_absolute = np.array([pos[0], pos[1], ori[2]])
#     return pos_absolute

def get_joints_angle():
    """return the pose of the robot joints relative to world coordination:  """
    res, ang_joint1 = vrep.simxGetJointPosition(clientID, joint1ID, BLOCKING)
    res, ang_joint2 = vrep.simxGetJointPosition(clientID, joint2ID, BLOCKING)
    res, ang_joint3 = vrep.simxGetJointPosition(clientID, joint3ID, BLOCKING)
    res, ang_joint4 = vrep.simxGetJointPosition(clientID, joint4ID, BLOCKING)
    res, ang_joint5 = vrep.simxGetJointPosition(clientID, joint5ID, BLOCKING)
    res, ang_joint6 = vrep.simxGetJointPosition(clientID, joint6ID, BLOCKING)
    return ang_joint1, ang_joint2, ang_joint3, ang_joint4, ang_joint5, ang_joint6

def get_joints_pose3d():
    res, pos_joints1 = vrep.simxGetObjectPosition(clientID, joint1ID, -1, BLOCKING)
    res, pos_joints2 = vrep.simxGetObjectPosition(clientID, joint2ID, -1, BLOCKING)
    res, pos_joints3 = vrep.simxGetObjectPosition(clientID, joint3ID, -1, BLOCKING)
    res, pos_joints4 = vrep.simxGetObjectPosition(clientID, joint4ID, -1, BLOCKING)
    res, pos_joints5 = vrep.simxGetObjectPosition(clientID, joint5ID, -1, BLOCKING)
    res, pos_joints6 = vrep.simxGetObjectPosition(clientID, joint6ID, -1, BLOCKING)
    return pos_joints1, pos_joints2, pos_joints3, pos_joints4, pos_joints5, pos_joints6


def get_robot_pose3d():
    """return the pose of the robot relative to world coordination: """
    res, pos_robot_end = vrep.simxGetObjectPosition(clientID, robotEndID, -1, BLOCKING)
    res, ori_robot_end = vrep.simxGetObjectOrientation(clientID, robotEndID, -1, BLOCKING)
    return pos_robot_end, ori_robot_end

def get_novel_pose3d():
    """return the pose of the people novel relative to world coordination"""
    res, pos_novel = vrep.simxGetObjectPosition(clientID, novelID, -1, BLOCKING)
    res, ori_novel = vrep.simxGetObjectOrientation(clientID, novelID, -1, BLOCKING)
    return pos_novel, ori_novel


# def get_relative_pose2d():
#     """ return the pose of the people relative to robot:  [ x(m), y(m), Theta(rad) ] """
#     global pos_relative
#     res, pos = vrep.simxGetObjectPosition(clientID, peopleID, robotID, MODE)
#     # print(pos)
#     res, ori = vrep.simxGetObjectOrientation(clientID, peopleID, robotID, MODE)
#     pos_relative = np.array([pos[0], pos[1], ori[2]])
#     return pos_relative


# def if_collision():
#     """ judge if collision happens"""
#     res, collision = vrep.simxReadCollision(clientID, collisionID, BUFFER)
#     if collision == 1:
#         print("Collision!")
#     return collision





# def move_wheels(v_left, v_right):
#     """ move the wheels. Input: Angular velocities in rad/s """
#     vrep.simxSetJointTargetVelocity(clientID, left_motorID, v_left, STREAMING)
#     vrep.simxSetJointTargetVelocity(clientID, right_motorID, v_right,STREAMING)
#     depthData = []
#     for i in range(4):
#         time.sleep(config.time_step)
#         depthData1 = fetch_kinect()
#         depthData.append(depthData1)
#     return depthData

def move_joints(a_joint1, a_joint2, a_joint3, a_joint4, a_joint5, a_joint6):

    """ move the joints. Input: Angular velocities in rad/s """

    # print('start j1-j6: ', a_joint1, a_joint2, a_joint3, a_joint4, a_joint5, a_joint6)

    # vrep.simxPauseSimulation(clientID, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint1ID, a_joint1, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint2ID, a_joint2, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint3ID, a_joint3, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint4ID, a_joint4, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint5ID, a_joint5, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint6ID, a_joint6, ONESHOT)
    # vrep.simxStartSimulation(clientID, ONESHOT)

    vrep.simxPauseCommunication(clientID, True)
    vrep.simxSetJointTargetPosition(clientID, joint1ID, a_joint1, ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint2ID, a_joint2, ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint3ID, a_joint3, ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint4ID, a_joint4, ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint5ID, a_joint5, ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint6ID, a_joint6, ONESHOT)
    vrep.simxPauseCommunication(clientID, False)


    # stay_sync()
    # time.sleep(5)

    # res, j1 = vrep.simxGetJointPosition(clientID, joint1ID, BLOCKING)
    # res, j2 = vrep.simxGetJointPosition(clientID, joint2ID, BLOCKING)
    # res, j3 = vrep.simxGetJointPosition(clientID, joint3ID, BLOCKING)
    # res, j4 = vrep.simxGetJointPosition(clientID, joint4ID, BLOCKING)
    # res, j5 = vrep.simxGetJointPosition(clientID, joint5ID, BLOCKING)
    # res, j6 = vrep.simxGetJointPosition(clientID, joint6ID, BLOCKING)
    #
    #
    # #
    # print('j1-j6: ', j1, j2, j3, j4, j5, j6)


    # depthData = []
    # for i in range(4):
    #     time.sleep(config.time_step)
    #     depthData1 = fetch_kinect()
    #     depthData.append(depthData1)
    # return depthData

    return

#def stop_motion():
#    """ stop the base wheels """
#    vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
#    vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)
#    return

# def stop_motion():
#     """ stop the base joints """
#     vrep.simxSetJointTargetVelocity(clientID, joint1ID, 0, STREAMING)
#     vrep.simxSetJointTargetVelocity(clientID, joint2ID, 0, STREAMING)
#     vrep.simxSetJointTargetVelocity(clientID, joint3ID, 0, STREAMING)
#     vrep.simxSetJointTargetVelocity(clientID, joint4ID, 0, STREAMING)
#     vrep.simxSetJointTargetVelocity(clientID, joint5ID, 0, STREAMING)
#     vrep.simxSetJointTargetVelocity(clientID, joint6ID, 0, STREAMING)
#     return

def reset_robot():
    """ reset the robot to the zero point"""
    # vrep.simxPauseSimulation(clientID, ONESHOT)
    # print('time delay')
    # vrep.simxSetJointTargetPosition(clientID, joint1ID, 0., ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint2ID, 90./rag2deg, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint3ID, 0., ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint4ID, 90./rag2deg, ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint5ID, 0., ONESHOT)
    # vrep.simxSetJointTargetPosition(clientID, joint6ID, 90./rag2deg, ONESHOT)
    # time.sleep(10)
    # vrep.simxStartSimulation(clientID, ONESHOT)

    vrep.simxPauseCommunication(clientID, True)
    vrep.simxSetJointTargetPosition(clientID, joint1ID, 0., ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint2ID, 0., ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint3ID, 0., ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint4ID, 0., ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint5ID, 0., ONESHOT)
    vrep.simxSetJointTargetPosition(clientID, joint6ID, 0., ONESHOT)
    vrep.simxPauseCommunication(clientID, False)

    # stay_sync()

    # print('time delay')
    # time.sleep(1)
    #
    #
    # res, j1 = vrep.simxGetJointPosition(clientID, joint1ID, BLOCKING)
    # res, j2 = vrep.simxGetJointPosition(clientID, joint2ID, BLOCKING)
    # res, j3 = vrep.simxGetJointPosition(clientID, joint3ID, BLOCKING)
    # res, j4 = vrep.simxGetJointPosition(clientID, joint4ID, BLOCKING)
    # res, j5 = vrep.simxGetJointPosition(clientID, joint5ID, BLOCKING)
    # res, j6 = vrep.simxGetJointPosition(clientID, joint6ID, BLOCKING)
    #
    # print('j1-j6: ', j1, j2, j3, j4, j5, j6)

    return

def stay_sync():
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
    return
# def if_in_range():
