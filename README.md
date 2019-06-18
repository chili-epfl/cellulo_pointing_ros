# Cellulo pointing gestures package

## Description

> This ROS package implements the method presented by [Goromov et al](https://ieeexplore.ieee.org/document/8594174) in order to guide several Cellulo robots, thanks to the informations of the MetaMotionR1 from Metawear ([Hardware datasheet](https://mbientlab.com/documents/MetaMotionR-PS3.pdf), [Software documentation](https://mbientlab.com/cppdocs/latest/index.html)). It uses the ROS framework to communicate between the different components.
It also include the node *ros_metawear_simple* which publishes the data of the sensor fusion from the IMU. 


## Installation

* This program run in python 2.7.
* Clone this repository inside your desired catkin_ws/src then build it.


### Dependencies

* Python packages: Numpy, Scipy, PyKDL, transforms3D.
* ROS: Already included messages.
* For the streaming node, one need also: mbientlab.metawear package


## Utilisation

> Command to launch the node: *rosrun cellulo_pointing_ros cellulo_pointing.py _robots_MAC:=00_06_66_E7_8E_6B 00_06_66_E7_8E_6A _address_imu_r=E5:4D:16:18:CD:90 _robot_nbr:=2 _user_name:=Lucas*

The last parameter is optionnal and can be used to retrieve morphologicals parameter's for different user saved earlier.
Please find details of the implementations in the pdf in the folder.

> Command to launch the streaming node: *rosrun cellulo_pointing_ros ros_metawear_simple.py _address:=E5:4D:16:18:CD:90*

*_address* is the MAC address of the IMU.

In the launch folder there is a ros launch file example. It start 3 cellulo nodes and the pointing node. 

### Human model:
Two different libraries where used to extract the skeleton of the user: [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) and [pifpaf](https://github.com/vita-epfl/openpifpaf). 
Then "HumanParam/GetHumanParam.py" translates the keypoints obtained in values for the human model and save it in a json file.

### It subscribes to: 

* */metawear_ros_E5_4D_16_18_CD_90/rotation*: gives orientation of IMU (quaternion).
* */tf* (paper_frame): gives robot's position in its paper frame.

### It publishes to:

* */cellulo_node_00_06_66_E7_8E_6B/setGoalPosition*: send destination to robot.
* */cellulo_node_00_06_66_E7_8E_6B/clearTracking*: stop the robot.
* */cellulo_node_00_06_66_E7_8E_6B/setVisualEffect*: change color of the robot's led.


## Reference
@inproceedings{gromov2018robot,
  author = {Gromov, Boris and Gambardella, Luca and Giusti, Alessandro},
  title = {Robot Identification and Localization with Pointing Gestures},
  booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year = {2018},
  pages = {3921-3928},
  keywords = {Robot sensing systems;Robot kinematics;Solid modeling;Manipulators;Drones;Three-dimensional displays},
  doi = {10.1109/IROS.2018.8594174},
  issn = {2153-0866},
  month = oct,
  video = {https://youtu.be/VaQ3aZBf_uE},
}