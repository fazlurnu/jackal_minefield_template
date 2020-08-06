#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad, rad2deg
from math import atan2, sqrt, pow
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from metal_detector_msgs.msg._Coil import Coil
from tf import transformations

# add self made libraries
from grid_creator import create_waypoints
from control import set_limit

# read/write stuff on screen
std = None

radStep = deg2rad(15)
linStep = 0.1
transformer = None
transListener = None

# Robot data
robotTwist = Twist()
robotPose = PoseStamped()
robotOdom = Odometry()
leftCoilPose = PoseStamped()
rightCoilPose = PoseStamped()

# Target pose
targetPose = PoseStamped()
distanceTolerance = 0.1
headingTolerance = 0.3
mineSent = False

# Create waypoints
targetList = create_waypoints((0,0), 3, 4, 0.5)
targetCounter = 0

#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
coils = Coil()

######################### AUXILIARY FUNCTIONS ############################

# Get a transformation matrix from a geometry_msgs/Pose
def matrix_from_pose_msg(pose):
    t = transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    r = transformations.quaternion_matrix(q)
    return transformations.concatenate_matrices(t, r)

# Get a geometry_msgs/Pose from a transformation matrix
def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position.x = transformation[0][3]
    msg.position.y = transformation[1][3]
    msg.position.z = transformation[2][3]
    q = transformations.quaternion_from_matrix(transformation)
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    return msg

######################### GETTING POSES ############################

def updateRobotPose():
    global robotPose
    global robotOdom
    # This function does not get the true robot pose, but only the pose of 'base_link' in the TF
    # you should replace it by the robot pose resulting from a good localization process 

    robotPose = PoseStamped()
    robotPose.pose = robotOdom.pose.pose

def updateLeftCoilPoseManually(referencePose):
    global transListener, leftCoilPose

    now = rospy.Time.now()
    # Get left coil pose in relation to robot
    try:
        transListener.waitForTransform('base_link', 'left_coil', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('base_link', 'left_coil', now)
    except:
        return

    localCoil_Mat = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

    # Use reference robot pose
    robot_Mat = matrix_from_pose_msg(referencePose)

    # Compute corrected coil pose
    corrected_Mat = np.dot(robot_Mat, localCoil_Mat)

    leftCoilPose = PoseStamped()
    leftCoilPose.pose = pose_msg_from_matrix(corrected_Mat)

def updateRightCoilPoseManually(referencePose):
    global transListener, rightCoilPose

    now = rospy.Time.now()
    # Get left coil pose in relation to robot
    try:
        transListener.waitForTransform('base_link', 'right_coil', now, rospy.Duration(2.0))    
        (trans,rot) = transListener.lookupTransform('base_link', 'right_coil', now)
    except:
        return

    localCoil_Mat = transformations.concatenate_matrices(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

    # Use reference robot pose
    robot_Mat = matrix_from_pose_msg(referencePose)

    # Compute corrected coil pose
    corrected_Mat = np.dot(robot_Mat, localCoil_Mat)

    rightCoilPose = PoseStamped()
    rightCoilPose.pose = pose_msg_from_matrix(corrected_Mat)

def getCenterOfCoilsPose(leftCoilPose, rightCoilPose):
    centerOfCoils = PoseStamped()
    minePose.pose.position.x = (leftCoilPose.pose.position.x + rightCoilPose.pose.position.x)/2
    minePose.pose.position.y = (leftCoilPose.pose.position.y + rightCoilPose.pose.position.y)/2

    return centerOfCoils

def toMinefieldPose(inputPose):
    outputPose = PoseStamped()
    outputPose.pose.position.x = -inputPose.pose.position.y
    outputPose.pose.position.y = inputPose.pose.position.x

    return outputPose

# Send mine position to HRATC Framework
def sendMine():
    global transListener

    minePose = PoseStamped()

    ## It is better to compute the coil pose in relation to a corrected robot pose
    updateRobotPose()
    updateLeftCoilPoseManually(robotPose.pose)
    updateRightCoilPoseManually(robotPose.pose)

    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)

    if (coils.left_coil > coils.right_coil):
        minePose = toMinefieldPose(leftCoilPose)
    elif (coils.right_coil > coils.left_coil):
        minePose = toMinefieldPose(rightCoilPose)
    else:
        centerOfCoils = getCenterOfCoilsPose(leftCoilPose, rightCoilPose)
        minePose = toMinefieldPose(centerOfCoils)
    
    pubMine.publish(minePose)

######################### CALLBACKS ############################

# Laser range-finder callback
def receiveLaserHokuyo(LaserNow):
    global laserInfoHokuyo 
    laserInfoHokuyo = LaserNow

# IMU data callback
def receiveImu(ImuNow):
    global imuInfo 
    imuInfo = ImuNow

# Odom data callback
def receiveOdom(OdomNow):
    global robotOdom
    robotOdom = OdomNow

# Mine Detection Callback
def receiveCoilSignal(actualCoil):
    global coils
    coils = actualCoil

    updateRobotPose() 
    updateLeftCoilPoseManually(robotPose.pose)
    updateRightCoilPoseManually(robotPose.pose)

######################### CURSES STUFF ############################

# Printing data on screen
def showStats():

    if std == None:
        return

    std.clear()
    std.addstr(0,0,"Press Esc to Quit...")
    std.addstr(1,0,"Linear:")
    std.addstr(2, 0, "{} \t {} \t {}".format(robotTwist.linear.x,robotTwist.linear.y,robotTwist.linear.z))
    std.addstr(4,0,"Angular:")
    std.addstr(5, 0, "{} \t {} \t {}".format(robotTwist.angular.x,robotTwist.angular.y,robotTwist.angular.z))
    std.addstr(7,0,"Robot Position:")
    std.addstr(8, 0, "{} \t {} \t {}".format(robotPose.pose.position.x, robotPose.pose.position.y, robotPose.pose.position.z))
    std.addstr(9,0,"Coil Position:")
    std.addstr(10, 0, "left: {} \t {} \t {}".format(leftCoilPose.pose.position.x, leftCoilPose.pose.position.y, leftCoilPose.pose.position.z))
    std.addstr(11, 0, "right: {} \t {} \t {}".format(rightCoilPose.pose.position.x, rightCoilPose.pose.position.y, rightCoilPose.pose.position.z))
    std.addstr(12,0,"Target Position:")
    std.addstr(13, 0, "{} \t {} \t {}".format(targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z))
    std.addstr(14,0,"Distance:")
    std.addstr(15, 0, "{}".format(getDistanceToTarget()))
    std.addstr(16,0,"Heading, Heading Target:")
    std.addstr(17, 0, "{} \t {}".format(getYaw(), getHeadingTarget()))

    std.addstr(18, 0, "Coils readings: l: {} \t r: {}".format(coils.left_coil, coils.right_coil))
    #std.addstr(19, 0, "IMU Quaternion w: {:0.4f} x: {:0.4f} y: {:0.4f} z: {:0.4f} ".format(imuInfo.orientation.w, imuInfo.orientation.x, imuInfo.orientation.y, imuInfo.orientation.z))
    if laserInfo.ranges != []:
        std.addstr(20, 0 , "Laser Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfo.ranges), min(laserInfo.ranges), max(laserInfo.ranges)))
    if laserInfoHokuyo.ranges != []:
        std.addstr(21, 0 , "Laser Hokuyo Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfoHokuyo.ranges), min(laserInfoHokuyo.ranges), max(laserInfoHokuyo.ranges)))

    std.refresh()

# Basic control
def KeyCheck(stdscr):
    global targetCounter

    stdscr.keypad(True)
    stdscr.nodelay(True)
    
    k = None
    global std
    std = stdscr

    #publishing topics
    pubVel   = rospy.Publisher('/cmd_vel', Twist)

    #control param
    linear_speed_lower_limit = 0.5
    linear_speed_upper_limit = 1.5
    angular_speed_lower_limit = 0.2
    angular_speed_upper_limit = 2

    # While 'Esc' is not pressed
    while k != chr(27):
        # Check no key
        try:
            k = stdscr.getkey()
        except:
            k = None

        if k == "x":
            sendMine()

        currentTarget = targetList[targetCounter]
        setTargetPose(currentTarget[0], currentTarget[1])

        distance = getDistanceToTarget()
        headingTarget = getHeadingTarget()
        headingDiff = getHeadingDiff()

        kp_angular = -0.03
        kp_linear = 0.8

        if(headingDiff > headingTolerance):
            robotTwist.angular.z = set_limit(kp_angular*headingDiff, -angular_speed_lower_limit, -angular_speed_upper_limit)
            robotTwist.linear.x = 0
        elif(headingDiff < -headingTolerance):
            robotTwist.angular.z = set_limit(kp_angular*headingDiff, angular_speed_upper_limit, angular_speed_lower_limit)
            robotTwist.linear.x = 0
        else:
            robotTwist.angular.z = kp_angular*headingDiff
            if(distance > distanceTolerance):
                robotTwist.linear.x = set_limit(kp_linear * distance, linear_speed_upper_limit, linear_speed_lower_limit)
            else:
                targetCounter += 1
                robotTwist.linear.x = 0
        
        pubVel.publish(robotTwist)

        showStats()

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")

# Navigation
def setTargetPose(x, y):
    global targetPose

    targetPose.pose.position.x = x
    targetPose.pose.position.y = y

def getHeadingTarget():
    global targetPose
    global robotPose

    y1 = robotPose.pose.position.y
    y2 = targetPose.pose.position.y

    x1 = robotPose.pose.position.x
    x2 = targetPose.pose.position.x

    headingTarget = atan2((y2-y1), (x2-x1))

    return rad2deg(headingTarget)

def getYaw():
    global robotPose

    quaternion = (
        robotPose.pose.orientation.x,
        robotPose.pose.orientation.y,
        robotPose.pose.orientation.z,
        robotPose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    return rad2deg(yaw)

def getHeadingDiff():
    
    headingDiff = getYaw() - getHeadingTarget()
    return headingDiff


def getDistanceToTarget():
    global targetPose
    global robotPose

    y1 = robotPose.pose.position.y
    y2 = targetPose.pose.position.y
    diffY = y2-y1

    x1 = robotPose.pose.position.x
    x2 = targetPose.pose.position.x
    diffX = x2-x1

    distance = sqrt(pow(diffY,2) + pow(diffX,2))

    return distance

######################### MAIN ############################

def spin():
    rospy.spin()
    
def StartControl():
    wrapper(KeyCheck)

if __name__ == '__main__':
    # Initialize client node
    rospy.init_node('client')

    transListener = tf.TransformListener()

    # Subscribing to all these topics to bring the robot or simulation to live data
    rospy.Subscriber("/odometry/filtered", Odometry, receiveOdom)
    rospy.Subscriber("/coils", Coil, receiveCoilSignal, queue_size = 1)
    rospy.Subscriber("/imu/data", Imu, receiveImu)
    rospy.Subscriber("/scan_hokuyo_jackal", LaserScan, receiveLaserHokuyo)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()