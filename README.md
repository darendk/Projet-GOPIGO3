# GoPiGo3 Autonomous Navigation Robot.
   The main objective of the Gopigo Robot is to under-go a maze autonomously using ultrasound.
# Collaborators:
 - BALOOMOODY DAREN 
 - Vibhakarsingh BEEHARRY
 - KRITIKA BISSESSUR
 - HACHIM HASANI BACAR
   
## This repository contains the necessary setup and code to control the GoPiGo3 robot autonomously using ROS (Robot Operating System) and three ultrasonic sensors.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Setup Instructions](#setup-instructions)
  - [Install ROS](#install-ros)
  - [Create ROS Workspace](#create-ros-workspace)
  - [Create ROS Package](#create-ros-package)
  - [Create and Configure Nodes](#create-and-configure-nodes)
- [Running the Nodes](#running-the-nodes)
- [Teleoperation](#teleoperation)

## Prerequisites
Ensure you have the following installed on your system:
- ROS (Robot Operating System) Noetic
- Python 3

## Setup Instructions
### Install ROS
Install ROS Noetic if it is not already installed:

```sh
sudo apt update
sudo apt upgrade
sudo apt install ros-noetic-desktop-full
```

### Ensure your ROS environment is set up by sourcing the setup file:
```sh 
source /opt/ros/<ros_noetic>/setup.bash
```

### Create the Workspace Directory
 ```sh
mkdir -p ~/catkin_ws/src
```

### Initialize the Workspace
Navigate to the src directory and initialize the workspace:
```sh
cd ~/catkin_ws/src
catkin_init_workspace
```

### Build the Workspace
Navigate back to the root of your workspace and build it:
```sh
cd ~/catkin_ws
catkin_make
```
This command builds the workspace and creates several directories (build, devel, etc.).


## To install the packages required for a GoPiGo3 robot, follow these steps. GoPiGo3 is a robot platform by Dexter Industries, and setting it up involves installing the necessary software and dependencies on your Raspberry Pi.

### Clone the GoPiGo3 Repository
Navigate to the home directory and Clone the GoPiGo3 repository:
```sh
git clone https://github.com/DexterInd/GoPiGo3.git
```

### Install GoPiGo3 Software
Navigate to the Software directory and run the installation script:
```sh
sudo bash install.sh
```

### Install the GoPiGo3 Python Package
```sh
sudo pip3 install gopigo3
```

### Installing Ultrasonic Sensor Library
```sh
sudo pip3 install easygopigo3
```

### To verify your GoPiGo3 is working with ROS, you can launch a ROS node:
Launch the ROS master:
 ```sh
roscore
```

### Open a new terminal and source the workspace:
```sh
source ~/catkin_ws/devel/setup.bash
```

Run a GoPiGo3 node:
```sh
rosrun gopigo3_node gopigo3_node
```
By following these steps, you should be able to install and set up all necessary packages for your GoPiGo3 robot.


Once all library and packages has been installed and tested we can now run the robot using the steps below.
## Running the GoPiGo Robot
### Launch the Differential Drive Node:

#### Step 1 : Open a terminal and navigate to your catkin workspace:
```sh
cd ~/catkin_ws
```
Changes the directory to  catkin workspace. 
This is the root directory for our ROS workspace where all packages and build files are 
located.


#### Step 2 : Source the setup file to ensure all environment variables are correctly defined:
```sh
source devel/setup.bash
```
Sources the setup file to set environment variables. 
This command ensures that the terminal session is aware of the ROS packages and paths defined 
in your workspace.


#### Step 3 : Launch the differential drive node, which controls the motors of the GoPiGo robot:
```sh
roslaunch bringup car_differential_drive.launch
```
Launches a ROS launch file. This specific launch file typically initializes the nodes required
for controlling the differential drive mechanism of the GoPiGo robot. It may include starting 
motor control nodes, sensor nodes, and other necessary components.


#### Step 4 : Run the GoPiGo Control Script:
```sh
python3 gopigo.py
``` 
It runs the Python script gopigo.py, which contains the logic for controlling the GoPiGo robot. 
This script handle sensor readings, motor commands, and overall navigation logic.


#### Step 5 : To terminate the session
```sh
ctrl + z
```
This command will terminate the session and the robot will stop automatically.
