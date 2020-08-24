# jackal_minefield_template

This repository contains the navigation algorithm to control [Jackal](https://github.com/jackal) for [HRATC 2017 Challenge](http://www.inf.ufrgs.br/hratc2017/HRATC2017/Welcome.html).

1. When a landmine is detected, new waypoints are created around the mine as a guidance to avoid the mine.
2. A [bug-algorithm](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiUvsGZ4ZfrAhXkILcAHVzPDp0QFjASegQIAxAC&url=https%3A%2F%2Fwww.cs.cmu.edu%2F~motionplanning%2Flecture%2FChap2-Bug-Alg_howie.pdf&usg=AOvVaw2kffae1giEB-SpLVJ3fw-K) is used for the obstacle avoidance

Read the full documentation on [my website](https://fazlurnu.com/2020/08/24/humanitarian-robotics-autonomous-landmine-detection-rover/)!
### Results:

**1. Waypoint Following + Landmine Detection, without Obstalces**

![Jackal](https://github.com/fazlurnu/jackal_minefield_template/blob/master/JackalMinefield.gif)

**2. Waypoint Following + Landmine Detection + Obstacle Avoidance**

![JackalwithObstacleAvoidance](https://github.com/fazlurnu/jackal_minefield_template/blob/master/withObstacleAvoidance.gif)

For the full video, click [here](https://www.youtube.com/watch?v=7qwAT6k6M70)

### Install Dependencies:
This is *manual* way to install necessary packages :
```
sudo apt-get install ros-kinetic-robot-localization ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-diff-drive-controller ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-plugins             ros-kinetic-lms1xx ros-kinetic-pointgrey-camera-description ros-kinetic-roslint ros-kinetic-amcl ros-kinetic-gmapping      ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-message-runtime ros-kinetic-topic-tools ros-kinetic-teleop-twist-joy
```

### Create a workspace and clone sources

Jackal Sources:
```
mkdir -p jackal_minefield_ws/src; cd jackal_ws/src; catkin_init_workspace
git clone https://github.com/jackal/jackal.git
git clone https://github.com/jackal/jackal_desktop.git
git clone https://github.com/fazlurnu/jackal_minefield_simulator.git
git clone https://github.com/fazlurnu/jackal_minefield_template.git
```
HRATC 2017 Sources:
```
git clone https://github.com/phir2-lab/metal_detector_msgs.git
```

### Build and source

```
cd jackal_minefield_ws; catkin_make; source devel/setup.bash
```
### Start simulation : 

In terminal 1 :
```
roslaunch jackal_gazebo minefield.launch
```
In terminal 2 :
```
rosrun jackal_minefield_template navigation.py
```
