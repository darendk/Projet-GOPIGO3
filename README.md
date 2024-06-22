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
sudo apt update
sudo apt install ros-noetic-desktop-full


### Source the ROS setup script:
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
