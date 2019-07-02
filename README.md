# Suii_control
This repository is a meta package that contain the packages for the high level control of the Suii's system. The main goal of this package is to plan, optimize and execute tasks in a sequential order. The four main packages in this meta package includes;
* suii_protocol
* suii_task_manager
* suii_mux_manager_comm
* suii_task_executor

In the wiki links bellow, you can find more in-depth information about this package.

* [Software Architecture](https://github.com/RoboHubEindhoven/suii_control/wiki/Software-Architecture)
* [State Machine](https://github.com/RoboHubEindhoven/suii_control/wiki/State-Machine)
* [Communication](https://github.com/RoboHubEindhoven/suii_control/wiki/Communication)

## Getting started

For this package, it is assumed that you have ubuntu 16.04 LTS with ROS Kinetic installed on your PC.

### Prerequisites

This package requires the following packages to be installed:

* [RoboCup@work refbox comm](https://github.com/industrial-robotics/atwork_refbox_comm): To install RoboCup@Work referee box. Click [here](https://github.com/industrial-robotics/atwork_central_factory_hub) For more info.
* [robomis](https://github.com/RoboHubEindhoven/robomis): For mission overview and creating custom missions
* [mission_planner_comm](https://github.com/RoboHubEindhoven/mission_planner_comm): To use custom mission data-types and messages

The following packages could be installed:

* [suii](https://github.com/RoboHubEindhoven/suii): If you want to perform an overall system test

### Usage

Enter the commands bellow on your Ubuntu terminal to run the following packages;

* Suii Refbox Client package: 
```
roslaunch suii_refbox_client suii_refbox_client.launch
```
* Task Manager package: 
```
roslaunch suii_task_manager suii_task_manager.launch
```
* Task Executor package: 
```
roslaunch suii_task_executor suii_task_executor.launch
```
## Authors

**Thierry Zinkeng and Thanh** - *in the name of RoboHub Eindhoven* - [RoboHub Eindhoven website](https://robohub-eindhoven.nl/)
