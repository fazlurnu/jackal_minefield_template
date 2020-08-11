#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy, os, sys, curses, time, cv2, tf
import numpy as np
import copy
from numpy import deg2rad, rad2deg
from math import atan2, sqrt, pow, sin, cos
from curses import wrapper
from threading import Thread
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import MarkerArray
from metal_detector_msgs.msg._Coil import Coil
from tf import transformations

# add self made libraries
from grid_creator import create_waypoints, create_target_lines
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
minePose = PoseStamped()
sendingMineGuess = False
robot2detectedMineDistance = 0

# Target pose
targetPose = PoseStamped()
distanceTolerance = 0.05
xTrackErrorTolerance = 0.05
headingTolerance = 1
mineGuessSent = False
newWaypointsSent = False

prevTargetPose = PoseStamped()
prevTargetPose.pose.position.x = robotPose.pose.position.x
prevTargetPose.pose.position.y = robotPose.pose.position.y

#control param
kp_angular = -0.03
kp_linear = 0.8

linear_speed_lower_limit = 0.5
linear_speed_upper_limit = 0.65
angular_speed_lower_limit = 0.15
angular_speed_upper_limit = 2

# Create waypoints
missionFinished = False
init_coordinate = (-3.6,-4.0)
width = 9
height = 8
spacing = 0.5
targetList = create_waypoints(init_coordinate, width, height, spacing)

# Rounding the mine
backward_speed = 2
mine_clearance = 1

# Markers
properMineMarkers = MarkerArray()
wrongMineMarkers = MarkerArray()
properMineMarkersSizePrev = 0
wrongMineMarkersSizePrev = 0

#laser information
laserInfo = LaserScan()
laserInfoHokuyo = LaserScan()

#Inertial Unit 
imuInfo = Imu()

#Metal detector data
coils = Coil()
coilValueMineDetected = 0.45
mineNotDetected = False
sendMineCounter = 0

# States
currentState = 0
states = {
    0: "Go To Waypoint",
    1: "Sending Mine Guess, Sending New Waypoint",
    2: "Sending Mine Guess, New Waypoint Sent",
    3: "Mine Guess Sent",
    4: "Avoid Obstacle",
    1000 : "State Unknown"
}

# Obstacle Parameters
regions = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

currentObstacleState = 0
obstacleStates = {
    0: "No Obstacles",
    1: "Turn Left",
    2: "Turn Right",
    3: "Follow Wall"
}

obstacleClearance = 1.25

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
    centerOfCoils.pose.position.x = (leftCoilPose.pose.position.x + rightCoilPose.pose.position.x)/2
    centerOfCoils.pose.position.y = (leftCoilPose.pose.position.y + rightCoilPose.pose.position.y)/2

    return centerOfCoils

def toMinefieldPose(inputPose):
    outputPose = PoseStamped()
    outputPose.pose.position.x = -inputPose.pose.position.y
    outputPose.pose.position.y = inputPose.pose.position.x

    return outputPose

# Send mine position to HRATC Framework
def sendMine():
    global transListener
    global minePose
    global mineGuessSent
    global sendMineCounter

    sendMineCounterTimeout = 20

    ## It is better to compute the coil pose in relation to a corrected robot pose
    updateRobotPose()
    updateLeftCoilPoseManually(robotPose.pose)
    updateRightCoilPoseManually(robotPose.pose)

    pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped, queue_size=1)

    if (coils.left_coil > coils.right_coil):
        minePose = toMinefieldPose(leftCoilPose)
    elif (coils.right_coil > coils.left_coil):
        minePose = toMinefieldPose(rightCoilPose)
    else:
        centerOfCoils = getCenterOfCoilsPose(leftCoilPose, rightCoilPose)
        minePose = toMinefieldPose(centerOfCoils)
    
    pubMine.publish(minePose)

    sendMineCounter += 1

    if(sendMineCounter > sendMineCounterTimeout):
        mineGuessSent = True
        sendMineCounter = 0

    time.sleep(0.5)

def getState():
    state_ = 1000

    if (mineNotDetected):
        state_ = 0
    else:
        if(not(mineGuessSent)):
            if(not(newWaypointsSent)):
                state_ = 1
            else:
                state_ = 2
        else:
            state_ = 3

    return state_

def getObstacleState():
    global regions
    
    currentObstacleState_ = 1000

    #obstacleStates = {
    #    0: "No Obstacles",
    #    1: "Turn Left",
    #    2: "Turn Right",
    #    3: "Follow Wall"
    #}

    state_description = ''

    frontObstacleFree = regions['front'] > obstacleClearance
    fleftObstacleFree = regions['fleft'] > obstacleClearance
    frightObstacleFree = regions['fright'] > obstacleClearance

    if frontObstacleFree and fleftObstacleFree and frightObstacleFree:
        # no obstacle
        currentObstacleState_ = 0

    elif not(frontObstacleFree) and fleftObstacleFree and frightObstacleFree:
        #obstacle front, turn left
        currentObstacleState_ = 1

    elif frontObstacleFree and fleftObstacleFree and not(frightObstacleFree):
        #wall on right side, follow wall
        currentObstacleState_ = 3

    elif frontObstacleFree and not(fleftObstacleFree) and frightObstacleFree:
        #wall on left side, follow wall
        currentObstacleState_ = 3

    elif not(frontObstacleFree) and fleftObstacleFree and not(frightObstacleFree):
        #wall on front and fright, turn left
        currentObstacleState_ = 1

    elif not(frontObstacleFree) and not(fleftObstacleFree) and frightObstacleFree:
        #wall on front and fleft, turn right
        currentObstacleState_ = 2

    elif not(frontObstacleFree) and not(fleftObstacleFree) and not(frightObstacleFree):
        #wall everywhere, turn left
        currentObstacleState_ = 1

    elif frontObstacleFree and not(fleftObstacleFree) and not(frightObstacleFree):
        #wall on left and right, go forward
        currentObstacleState_ = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    return currentObstacleState_
######################### CALLBACKS ############################

def receiveMineGuess(PoseGuess):
    global minePose

    minePose = PoseGuess

# Laser range-finder callback
def receiveLaserHokuyo(LaserNow):
    global laserInfoHokuyo 
    global regions

    laserInfoHokuyo = LaserNow

    regionSize = int(len(laserInfoHokuyo.ranges)/len(regions))

    regions = {
        'right':  min(min(laserInfoHokuyo.ranges[regionSize*0 : regionSize-1]), 10),
        'fright': min(min(laserInfoHokuyo.ranges[regionSize*1 : regionSize*2-1]), 10),
        'front':  min(min(laserInfoHokuyo.ranges[regionSize*2 : regionSize*3-1]), 10),
        'fleft':  min(min(laserInfoHokuyo.ranges[regionSize*3 : regionSize*4-1]), 10),
        'left':   min(min(laserInfoHokuyo.ranges[regionSize*4 : regionSize*5-1]), 10),
    }

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

def receiveWrongMineMarker(wrongMarkersInput):
    global wrongMineMarkers
    global wrongMineMarkersSizePrev
    global mineGuessSent

    wrongMineMarkers = wrongMarkersInput
    wrongMineMarkersSizeNow = len(wrongMineMarkers.markers)

    if (wrongMineMarkersSizePrev != wrongMineMarkersSizeNow):
        wrongMineMarkersSizePrev = wrongMineMarkersSizeNow
        mineGuessSent = True


def receiveProperMineMarker(properMarkersInput):
    global properMineMarkers
    global properMineMarkersSizePrev
    global mineGuessSent
    
    properMineMarkers = properMarkersInput
    properMineMarkersSizeNow = len(properMineMarkers.markers)

    if (properMineMarkersSizePrev != properMineMarkersSizeNow):
        properMineMarkersSizePrev = properMineMarkersSizeNow
        mineGuessSent = True

def addWaypointsAroundTheMine():
    global targetList
    global newWaypointsSent

    minePose_x = minePose.pose.position.y
    minePose_y = -minePose.pose.position.x
    currentTarget_x = targetPose.pose.position.x
    currentTarget_y = targetPose.pose.position.y

    currentTarget2detectedMineDistance = getDistance(minePose_x, minePose_y, currentTarget_x, currentTarget_y)

    WPnew = []
    # current target is close to mine
    if (currentTarget2detectedMineDistance > mine_clearance):
        heading = deg2rad(getHeading())
        currentTarget = (targetPose.pose.position.x, targetPose.pose.position.y)
        targetList.insert(0, currentTarget)

        for newWPAngle in range(180, 0, -90):

            newWPAngleRad = deg2rad(newWPAngle)
            x_new = minePose.pose.position.x + mine_clearance * sin(heading + newWPAngleRad)
            y_new = minePose.pose.position.y + mine_clearance * cos(heading + newWPAngleRad)
            newWP = (y_new, -x_new)
            
            print(newWPAngle, newWP)
            targetList.insert(0, newWP)

    newWaypointsSent = True

######################### CURSES STUFF ############################

# Printing data on screen
def showStats():
    if std == None:
        return

    std.clear()
    std.addstr(0,0,"Press Esc to Quit...")
    std.addstr(1,0,"Linear:")
    std.addstr(2, 0, "{} \t {} \t {}".format(robotTwist.linear.x,robotTwist.linear.y,robotTwist.linear.z))
    std.addstr(3,0,"Angular:")
    std.addstr(4, 0, "{} \t {} \t {}".format(robotTwist.angular.x,robotTwist.angular.y,robotTwist.angular.z))
    std.addstr(5,0,"Robot Position:")
    std.addstr(6, 0, "{} \t {} \t {}".format(robotPose.pose.position.x, robotPose.pose.position.y, robotPose.pose.position.z))
    std.addstr(7,0,"Coil Position:")
    std.addstr(8, 0, "left: {} \t {} \t {}".format(leftCoilPose.pose.position.x, leftCoilPose.pose.position.y, leftCoilPose.pose.position.z))
    std.addstr(9, 0, "right: {} \t {} \t {}".format(rightCoilPose.pose.position.x, rightCoilPose.pose.position.y, rightCoilPose.pose.position.z))
    std.addstr(10,0,"Target Position:")
    std.addstr(11, 0, "{} \t {} \t {}".format(targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z))
    std.addstr(12,0,"Prev Target Position:")
    std.addstr(13, 0, "{} \t {} \t {}".format(prevTargetPose.pose.position.x, prevTargetPose.pose.position.y, prevTargetPose.pose.position.z))
    std.addstr(14,0,"Distance, Cross Track Distance:")
    std.addstr(15, 0, "{} \t {}".format(getDistanceToTarget(), getCrossTrackDistance()))
    std.addstr(16,0,"Heading, Heading Target:")
    std.addstr(17, 0, "{} \t {}".format(getHeading(), getHeadingTarget()))
    std.addstr(18, 0, "Coils readings: l: {} \t r: {}".format(coils.left_coil, coils.right_coil))

    targetString = "Next Target: "
    for i in range (len(targetList)):
        targetString += str(targetList[i]) + ", "

    std.addstr(19, 0, targetString)

    std.addstr(22, 0, "Current State: {}, \tObstacle State: {}".format(states[getState()], obstacleStates[currentObstacleState]))
    #std.addstr(21, 0, "Wrong Detected Marker Size: {}".format(len(wrongMineMarkers.markers)))
    #std.addstr(19, 0, "IMU Quaternion w: {:0.4f} x: {:0.4f} y: {:0.4f} z: {:0.4f} ".format(imuInfo.orientation.w, imuInfo.orientation.x, imuInfo.orientation.y, imuInfo.orientation.z))
    if laserInfoHokuyo.ranges != []:
        std.addstr(23, 0 , "Laser Hokuyo Readings {} Range Min {:0.4f} Range Max {:0.4f}".format( len(laserInfoHokuyo.ranges), min(laserInfoHokuyo.ranges), max(laserInfoHokuyo.ranges)))

    std.refresh()

# Robot command function
def moveForward(speed):
    robotTwist.linear.x = speed
    robotTwist.angular.z = 0

def moveBackward(speed):
    robotTwist.linear.x = -speed
    robotTwist.angular.z = 0

def robotStop():
    robotTwist.linear.x = 0
    robotTwist.angular.z = 0

# Waypoint following algorithm
def goToWaypoint(headingDiff, distance):
    global missionFinished
    global mineGuessSent
    global newWaypointsSent
            
    # reset param
    mineGuessSent = False
    newWaypointsSent = False

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
            robotTwist.linear.x = 0

            if (len(targetList) > 0):
                currentTarget = targetList.pop(0)
                setTargetPose(currentTarget[0], currentTarget[1])
            else:
                missionFinished = True

def followWall():
    global robotTwist

    robotFootPrint = 0.6
    robotTwist.linear.x = 0.65
    kp_followWall = 1

    if (regions['fleft'] < robotFootPrint):
        robotTwist.angular.z = kp_followWall*(robotFootPrint - regions['fleft'])
    elif (regions['fright'] < robotFootPrint):
        robotTwist.angular.z = -kp_followWall*(robotFootPrint - regions['fright'])
    else:
        robotTwist.angular.z = 0

def turnLeft():
    global robotTwist

    robotTwist.linear.x = 0
    robotTwist.angular.z = 1

def turnRight():
    global robotTwist

    robotTwist.linear.x = 0
    robotTwist.angular.z = -1

# Basic control
def KeyCheck(stdscr):
    global targetList
    global robot2detectedMineDistance
    global mineNotDetected
    global currentState
    global currentObstacleState
    
    stdscr.keypad(True)
    stdscr.nodelay(True)
    
    k = None
    global std
    std = stdscr

    #publishing topics
    pubVel   = rospy.Publisher('/cmd_vel', Twist)

    currentTarget = targetList.pop(0)
    setTargetPose(currentTarget[0], currentTarget[1])

    counter = 0

    # While 'Esc' is not pressed
    while (k != chr(27) and not(missionFinished)):
        # Check no key
        try:
            k = stdscr.getkey()
        except:
            k = None

        if k == "x":
            sendMine()

        #obstacleStates = {
        #    0: "No Obstacles",
        #    1: "Turn Left",
        #    2: "Turn Right",
        #    3: "Follow Wall"
        #}

        # get control params
        distance = getDistanceToTarget()
        headingDiff = getHeadingDiff()
        xTrackDistance = getCrossTrackDistance()

        # get coil conditions
        leftCoilDetected = coils.left_coil < coilValueMineDetected
        rightCoilDetected = coils.right_coil < coilValueMineDetected
        mineNotDetected = leftCoilDetected and rightCoilDetected

        currentState = getState()

        currentObstacleState = getObstacleState()

        if (currentObstacleState == 0):
            if (currentState == 0):
                goToWaypoint(headingDiff, distance)
            
            elif (currentState == 1):
                robotStop()
                sendMine()

                addWaypointsAroundTheMine()
                    
                currentTarget = targetList.pop(0)
                setTargetPose(currentTarget[0], currentTarget[1], avoidingMine = True)

            elif (currentState == 2):
                robotStop()
                sendMine()

            elif (currentState == 3):
                moveBackward(backward_speed)

            else:
                robotStop()
        
        elif (currentObstacleState == 1):
            turnLeft()
        elif (currentObstacleState == 2):
            turnRight()
        elif (currentObstacleState == 3):
            followWall()
        else:
            robotStop()

        pubVel.publish(robotTwist)

        showStats()

    stdscr.keypad(False)
    rospy.signal_shutdown("Shutdown Competitor")

# Navigation
def setTargetPose(x, y, avoidingMine=False):
    global targetPose
    global prevTargetPose

    if(avoidingMine):
        prevTargetPose.pose.position.x = robotPose.pose.position.x
        prevTargetPose.pose.position.y = robotPose.pose.position.y
    else:
        prevTargetPose.pose.position.x = targetPose.pose.position.x
        prevTargetPose.pose.position.y = targetPose.pose.position.y

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

def getHeading():
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
    
    headingDiff = getHeading() - getHeadingTarget()
    return headingDiff

def getDistance(x1, y1, x2, y2):
    diffY = y2 - y1
    diffX = x2 - x1

    return sqrt(pow(diffY,2) + pow(diffX,2))

def getDistanceToTarget():
    global targetPose
    global robotPose

    y1 = robotPose.pose.position.y
    y2 = targetPose.pose.position.y

    x1 = robotPose.pose.position.x
    x2 = targetPose.pose.position.x

    distance = getDistance(x1, y1, x2, y2)

    return distance

def getCrossTrackDistance():
    y0 = robotPose.pose.position.y
    y1 = targetPose.pose.position.y
    y2 = prevTargetPose.pose.position.y

    x0 = robotPose.pose.position.x
    x1 = targetPose.pose.position.x
    x2 = prevTargetPose.pose.position.x

    numerator = (y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1
    denum = getDistance(x1, y1, x2, y2)

    return numerator/denum

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
    rospy.Subscriber("/HRATC_FW/set_mine", PoseStamped, receiveMineGuess, queue_size = 1)
    rospy.Subscriber("/judge/wronglyDetectedMines_marker", MarkerArray, receiveWrongMineMarker, queue_size = 1)
    rospy.Subscriber("/judge/properlyDetectedMines_marker", MarkerArray, receiveProperMineMarker, queue_size = 1)

    #Starting curses and ROS
    Thread(target = StartControl).start()
    Thread(target = spin).start()