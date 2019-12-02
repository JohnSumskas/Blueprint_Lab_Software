# ROS for the Reach5Mini

BlueprintLab ROS packages for interfacing with the Reach5Mini products via ROS.

## File Structure Overview

* **R5M_Examples (repository main directory):**  
    * **ROS/blueprintlab_reachsystem_ros_messages (required):** ROS package for storing ROS message types.
    * **ROS/reach5mini_ros_passthrough (required):** Bottom-end ROS to Serial passthrough script for R5M data transmission. Main script: `scripts/r5m_passthrough.py`
    * **Examples/ROS/keyboard_r5m_test:** Package to control joints on a Reach 5 Mini (R5M) using a keyboard as the input. Main script: `scripts/keyboard_r5m.py`
    * **Examples/ROS/r5m_ros_controller:** Top-end control interface for sending commands to the R5M ROS topics and receiving data from the R5M ROS topics. Main script: `scripts/r5m_control.py`




## Installation
To install the necessary components in this repository, follow the steps below.

### Top Side Installation 
**Ubuntu 18.04 (or Linux equivalent) top side** Should be similar if using Ubuntu 16.04 (or Linux equivalent), but use **apt-get** instead.

Install Python3 and dependencies.
```bash
sudo apt update
sudo apt install python3-dev python3-setuptools python3-pip python3-yaml
sudo python3 -m pip install --upgrade pip
sudo python3 -m pip install pyserial numpy 
```

#### Install Robot Operating System (ROS)

If using Ubuntu 18.04 (or Linux equivalent), follow the instructions at the link to install ROS Melodic: [http://wiki.ros.org/melodic/Installation](http://wiki.ros.org/melodic/Installation)

If using Ubuntu 16.04 (or Linux equivalent), follow the instructions at the link to install ROS Kinetic: [http://wiki.ros.org/kinetic/Installation](http://wiki.ros.org/kinetic/Installation)

#### Enable Python3 support for ROS
```bash
sudo pip3 install rospkg catkin_pkg
```

#### Workspace setup and compilation

To install the components from this git repository, follow the below steps to create the workspace and install:
```bash
cd ~
mkdir catkin_ws/src
```
Copy the ROS packages from this repository to the `~/catkin_ws/src/` folder. Then follow the below commands:
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
Note: You must source the file via the command `source ~/catkin_ws/devel/setup.bash` everytime you open a new terminal. You can make this permanent by adding the line to your `~/.bashrc` config file via command `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`

Now the installation is complete.


### Bottom Side Installation
Bottom-side installation is very similar to the top end. If you have a system image of the working system use that image.

If you do not have a system image, the following things should be noted:
1. If the bottom side system runs ARM-based cores (eg. Raspberry Pi, NVIDIA TX2), you will have to install an ARM-based version of ROS (see [http://wiki.ros.org/Installation/UbuntuARM](http://wiki.ros.org/Installation/UbuntuARM) for a clue).


## Usage

### ROS Message Type and Uses

The ROS package `blueprintlab_reachsystem_ros_messages` contains all of the ROS message types to use. To communicate over ROS between the nodes, you must import this package into your ROS nodes.

Each message has a style following the conventions: 1 timestamp container (ROS 'time' data type), 1 device_id container ('uint8' data type), and 1 or multiple value/data containers.

The current message types are as below:

* **single_float.msg** (import in python via `from blueprintlab_reachsystem_ros_messages.msg import single_float`): Message has a single 'float32' type. Currently used to process packet_ids: POSITION, VELOCITY, CURRENT, OPENLOOP.
* **single_int.msg** (import in python via `from blueprintlab_reachsystem_ros_messages.msg import single_int`): Message has a single 'int32' type. Currently used to process packet_ids: MODE.
* **request_list.msg** (import in python via `from blueprintlab_reachsystem_ros_messages.msg import request_list`): Message has an undefined length 'int32[]' array type. Currently used to process packet request lists.
* **generic.msg** (import in python via `from blueprintlab_reachsystem_ros_messages.msg import generic`): Message has a packet_id container of type 'uint8', and an undefined length 'float32[]' array type to store values. If the packet_id is supposed to transmit 'int' instead of 'float', then it must be converted to to a 'float' when transmitting on the ROS system and converted back to 'int' on the other end. Currently used to process any other packet_id not in the previous message lists.

The timestamp should be updated everytime you are about to publish the message, which could be used to track latency in the system.


### Current packages and scripts


#### blueprintlab_reachsystem_ros_messages 
ROS package for storing ROS message types. Must be compiled and installed to be able to use any other package from BlueprintLab.
How to run: Message container only. Does not run as a node.

#### reach5mini_ros_passthrough 
Bottom-end ROS to Serial passthrough script for R5M data transmission. Main script: `scripts/r5m_passthrough.py`
How to run: 

1. Via `rosrun`: `rosrun reach5mini_ros_passthrough r5m_passthrough.py <args>`. 
`<args>` is an optional parameter designed for running different instances of the package. `<args>` contains 2 parameters: R5M instance number and serial port address. Example usage with <args> is `rosrun reach5mini_ros_passthrough r5m_passthrough.py 0 /dev/ttyUSB0`. Without `<args>`, the instance number defaults to 0 and port defaults to `/dev/ttyUSB0`.

2. Via `roslaunch`: `roslaunch reach5mini_ros_passthrough run_reach5mini_passthrough.launch`.
This is using the launch file at location `reach5mini_ros_passthrough/launch/run_reach5mini_passthrough.launch`. This is designed to launch multiple instances of the `reach5mini_ros_passthrough` with preset user arguments in the same format as the above (rosrun) example. Edit the files to change the default values or enable the running of more than one instance of this package.

#### keyboard_r5m_test 
Package to control joints on a R5M using a keyboard as the input. Main script: `scripts/keyboard_r5m.py`
How to run: `rosrun keyboard_r5m_test keyboard_r5m.py`

#### r5m_ros_controller 
This is a template for a top-side control script. It can be cloned and modified by users. It is designed for sending commands or requests to the R5M ROS topics and receiving data from the R5M ROS topics. Main script: `scripts/r5m_control.py`
How to run:

1. Via `rosrun`: `rosrun r5m_ros_controller r5m_control.py <args>`. 
`<args>` is an optional parameter designed for running different instances of the package. `<args>` contains 1 parameter: R5M instance number. Example usage with <args> is `rosrun reach5mini_ros_passthrough r5m_passthrough.py 0`. Without `<args>`, the instance number defaults to 0.

2. Via `roslaunch`: `roslaunch r5m_ros_controller run_reach5mini_control.launch`.
This is using the launch file at location `r5m_ros_controller/launch/run_reach5mini_control.launch`. This is designed to launch multiple instances of the `r5m_ros_controller` with preset user arguments in the same format as the above (rosrun) example. Edit the files to change the default values or enable the running of more than one instance of this package.


