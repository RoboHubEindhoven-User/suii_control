# Suii_control
There are 4 main packages:
* suii_protocol
* suii_task_manager
* suii_mux_manager_comm
* suii_task_executor (otherwise known as: the Mux)

# Package dependencies
![alt text](https://github.com/RoboHubEindhoven/suii_control/blob/master/package_dependencies.png)
The package’s contents:
1. **suii_protocol**
- All the enums (which ID’s corresponds with what)
- Generate dictionaries for the enums + for the locations with instance IDs
2. **suii_mux_manager_comm**
- Our defined classes to share between Task Manager and Mux 
- YAML handler (because it is also shared)
3. **suii_task_manager**
- Converter to convert RefBox ROS msg to our own classes (the ones in suii_mux_manager_comm)
- TaskManager: optimizes and organizes task lists
4. **suii_task_executor** 
- TaskExecutor: makes service calls to other components to execute the tasks

# Communication
![alt text](https://github.com/RoboHubEindhoven/suii_control/blob/master/rqt_graph.png)
Topics:
- `/suii_refbox_client/task_info` where tasks are published to TaskManager
- `/suii_task_executor/input` where TaskManager sends tasks to Mux
- `/suii_task_executor/replan` where TaskManager receives error list (error handling not implemented at the moment)
- `/table_height` table height for object manipulation
