# Capstone Project | Team OSCAR

### Team Members:
 - Chris Kalle ([bdschrisk](https://github.com/bdschrisk)) | Team Lead, Perception
 - Ralph Fehrer ([fera0013](https://github.com/fera0013)) | Perception
 - Hideto Kimuta ([HidetoKimura](https://github.com/HidetoKimura)) | Systems integration
 - Carlos Arreaza ([carreaza](https://github.com/carreaza)) | Behavioral planning
 - Moe Elsadig ([moe-elsadig](https://github.com/moe-elsadig)) | Behavioral planning


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.

## Overview
This project is the culmination of five team members who share a bold vision, to create safe autonomous vehicles the world over.

### Meet Carla
**The Udacity self-driving car**
![Carla](https://github.com/bdschrisk/CarND-Capstone-Project/raw/master/imgs/udacity-carla.jpg)

*To learn more about Carla, see [here](https://medium.com/udacity/how-the-udacity-self-driving-car-works-575365270a40)*

Using the Robot Operating System (ROS), each team member has developed and maintained a core component of the infrastructure that is demanded by a truly autonomous vehicle.
The three core components of any good robot are the following:
 -  Perception
    Sensing the environment to perceive obstacles, traffic hazards as well as traffic lights and road signs
 -  Planning
    Route planning to a given goal state using data from localisation, perception and environment maps
 -  Control
    Actualising trajectories formed as part of planning, in order actuate the vehicle, through steering, throttle and brake commands


**ROS Node Architecture**
![Node architecture](https://github.com/bdschrisk/CarND-Capstone-Project/raw/master/docs/final-project-ros-graph-v2.png)

### Node Details
#### Traffic Light Recognition
Using information from the vehicle's current pose as well as sensing the environment through raw camera data, we are able to detect the presence of traffic lights and 
perform recognition to determine their current state.  This sensing informs downstream ROS nodes whether the vehicle should drive, stop or slow down.

Sensing is performed by two independent models, one for the [detection of objects](https://github.com/bdschrisk/CarND-Capstone-Project/perception/Traffic-Light-Detection.ipynb), in this case - traffic lights.  The other model takes the output of the first, and 
classifies the traffic lights according to their state, e.g. green, yellow or red.  This two-prongued approach to recognition provides a robust detection model in case of 
failure, as well as being "hot-swappable" when improved models are available.

The detection model, a MobileNet Single-Shot-Detector, is also able to detect other obstacles on the road such as other cars, bikes, pedestrians, etc.

Classification of traffic lights is performed by a KaNet deep neural network.  The KaNet is unique because it allows additional independent learning through layer divergence 
of target variables.  It is also extremely fast for inference - a necessary requirement for real-time recognition.

![KaNet model](https://github.com/bdschrisk/CarND-Capstone-Project/raw/master/imgs/kanet-model.png)

**Training**
Training the KaNet model is performed like any other multi-class classification problem, using 1 of K encoding and cross-entropy loss.
*(For more information on training the KaNet model, [see the notebook](https://github.com/bdschrisk/CarND-Capstone-Project/perception/TL-Classifier-KaNet.ipynb))*


### Installation 

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

