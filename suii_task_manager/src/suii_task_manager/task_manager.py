#! /usr/bin/env python
import rospy
from suii_msgs.srv import DistanceToGoal
from functools import cmp_to_key

from suii_mux_manager_comm.yaml_handler import YAMLHandler

from suii_mux_manager_comm.task_with_action import TaskWithAction 
from suii_mux_manager_comm.task import Task, TaskStatus
from suii_mux_manager_comm.task_list import TaskList
from suii_mux_manager_comm.action_list import ActionList
from suii_protocol.task_protocol import TaskProtocol
from suii_protocol.protocol.enum_task_action import TaskActionType
from suii_protocol.protocol.enum_task_type import TaskType
from suii_protocol.protocol.enum_location_identifier import LocationIdentifierType
from enum import Enum

## ===== Main class ==== ##
class TaskManager(object):
    MAX_HOLDING_CAPACITY = 3

    def __init__(self, holding_capacity=MAX_HOLDING_CAPACITY, yaml_path="", verbose=False):
        # Basic config
        self.holding_capacity = holding_capacity 
        self.yaml_path = yaml_path
        self.verbose = verbose

        # Here come the lists
        self.task_list = TaskList()                 # The current one
        self.task_list_last_updated = TaskList()    # Last replanned task list
        self.task_list_original = TaskList()        # Original task list from refbox

        self.replan_list = ActionList()             # Action list from mux
        self.error_index = -1                       # Error index

        # Output
        self.output_list = ActionList()             # The output list to send to mux

        self.task_list_type = -1                    # Type of the list (TRANSPORTATION/NAVIGATION)

        # Helpers
        self.transport_optimizer = TaskTransportOptimizer(holding_capacity, yaml_path, verbose)
        self.navigation_optimizer = TaskNavigationOptimizer(yaml_path, verbose)
        self.task_replanner = TaskReplanner(yaml_path, verbose)

    ## Call to initialize task_list when received data from refbox
    def initialize_list(self, task_list):
        # Save list type as the type of the first task
        self.task_list_type = task_list.task_list[0].type

        # Save the original + set the current one
        self.task_list_original = task_list.make_duplicate()
        self.task_list_last_updated = task_list.make_duplicate()
        self.task_list = task_list.make_duplicate()

    ## Initializing necessary data and then calls helpers
    def optimize_transport(self):
        # Make duplicates so i won't lose the task_list
        self.transport_optimizer.reinitialize_input(self.task_list.make_duplicate())
        self.transport_optimizer.optimize()
        # Points to output
        self.output_list = self.transport_optimizer.output_list

    def optimize_navigation(self):
        # Make duplicates so i won't lose the task_list
        self.navigation_optimizer.reinitialize_input(self.task_list.make_duplicate())
        self.navigation_optimizer.optimize()
        # Points to output
        self.output_list = self.navigation_optimizer.output_list

    def call_replanner(self):
        self.task_replanner.reinitialize_input(self.replan_list, self.task_list_last_updated, self.error_index)
        self.task_replanner.replan()
        # The new task_list
        self.task_list = self.task_replanner.output_list.make_duplicate()
        # Save the last replanned ones
        self.task_list_last_updated = self.task_replanner.output_list.make_duplicate()

    ## Interface that task_manager_handler calls to optimize list
    def optimize_list(self):
        if (self.task_list_type == int(TaskType.TRANSPORTATION)):
            self.optimize_transport()
            return True
        elif (self.task_list_type == int(TaskType.NAVIGATION)):
            self.optimize_navigation()
            return True
        return False

    ## Interface that task_manager_handler calls when received error from mux
    def replan(self):
        # Loggity log
        rospy.logwarn("Error @ task: ")
        rospy.logwarn(self.replan_list.task_list[self.error_index])

        # Get new task list
        rospy.loginfo("Making new task list...")
        self.call_replanner()

        # Optimize
        if (self.task_list_type == int(TaskType.TRANSPORTATION)):
            rospy.loginfo("Sending new task list to transport optimizer...")
            self.optimize_transport()
        elif (self.task_list_type == int(TaskType.NAVIGATION)):
            rospy.loginfo("Sending new task list to navigation optimizer...")
            self.optimize_navigation()
        
        # Loggity log
        rospy.loginfo("Replan finished")
        rospy.loginfo("##################")
        print(self.output_list)
    
    ## Clear all the lists
    def clear(self):
        self.task_list.clear_task()
        self.task_list_last_updated.clear_task()
        self.task_list_original.clear_task()
        self.output_list.clear_task()

## ===== Service clients ===== ##
class DistanceToGoalClient:
    @staticmethod
    def call_serv(x, y):
        # rospy.loginfo("Waiting for service...")
        # rospy.wait_for_service('get_distance')
        try:
            # rospy.loginfo("Calling 'get_distance'")
            distance_to_goal = rospy.ServiceProxy('get_distance', DistanceToGoal)
            res = distance_to_goal(x, y)
            return res.distance
        except rospy.ServiceException:
            return -1

## ===== TaskFormatter ===== ##
# Input: Data to make a TaskWithAction, The list to append to
# Output: A TaskWithAction object appended to the list
class TaskFormatter(object):
    @staticmethod
    def format_drive(dest, result):
        TaskFormatter.format_move_to_drive(dest, result)
        TaskFormatter.format_actual_drive(dest, result)

    @staticmethod
    def format_actual_drive(dest, result):
        twa = TaskWithAction()
        twa.set_destination(dest)
        twa.set_action(int(TaskActionType.DRIVE))
        result.append(twa)

    @staticmethod
    def format_pick_task(task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(int(TaskActionType.PICK))
        result.append(twa)
    
    @staticmethod
    def format_pick_from_robot(task, result):
        twa = TaskWithAction()
        twa.set_action(int(TaskActionType.PICK_FROM_ROBOT))
        twa.set_object(task.object)
        result.append(twa)

    @staticmethod
    def format_place_task (task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(int(TaskActionType.PLACE))
        result.append(twa)
    
    @staticmethod
    def format_place_to_robot(task, result):
        twa = TaskWithAction()
        twa.set_action(int(TaskActionType.PLACE_TO_ROBOT))
        twa.set_object(task.object)
        result.append(twa)

    @staticmethod
    def format_move_to_drive(task, result):
        twa = TaskWithAction()
        twa.set_action(int(TaskActionType.MOVE_TO_DRIVE))
        result.append(twa)

    @staticmethod
    def format_find_hole(task, result):
        twa = TaskWithAction()
        twa.set_object(task.object)
        twa.set_action(int(TaskActionType.FIND_HOLE))
        result.append(twa)

## ===== ErrorCase Enum ===== ##
# To keep track of what went wrong this time
class ErrorCase(Enum):
    CANNOT_DRIVE_TO_LOC = 1

## ===== TaskSorter ===== ##
# Sort list based on distances (and other things, later)
# (This object makes service calls to update distances)
class TaskSorter(object):
    def __init__(self, yaml_path, verbose=False):
        self.yaml_handler = YAMLHandler(path=yaml_path)
        self.yaml_handler.load()
        self.verbose = verbose
    
    def cmp_distance_src_dest(self, t1, t2):
        # -1 appears first, then 0, then 1
        # sorting order:
        #   - distance from source (asc)
        #   - distance from dest (asc)
        if ((t1.source_dist < t2.source_dist) or 
            ((t1.source_dist == t2.source_dist) and (t1.dest_dist < t2.dest_dist))):
            return -1
        elif ((t1.source_dist == t2.source_dist) and (t1.dest_dist == t2.dest_dist)):
            return 0
        else:
            return 1

    def cmp_distance_dest(self, t1, t2):
        if (t1.dest_dist < t2.dest_dist):
            return -1
        elif (t1.dest_dist == t2.dest_dist):
            return 0
        return 1

    def update_distances(self, task_list):
        for item in task_list.task_list:
            # index of the data: orientation [w,x,y,z]  - position [x,y,z]
            # Dist. from source
            if self.verbose:
                rospy.loginfo("Updating source_dist for task @ '%s'", item.source_str)
                rospy.loginfo("Calling service with x=%f, y=%f" % (position[0], position[1]))
            orientation, position = self.yaml_handler.get_pose_for(item.source_str)
            if position != None:
                item.source_dist = DistanceToGoalClient.call_serv(position[0], position[1])
                if (item.source_dist == -1):
                    return False

            # Dist. from dest.
            if self.verbose:
                rospy.loginfo("Updating dest_dist for task @ '%s'", item.destination_str)
                rospy.loginfo("Calling service with x=%f, y=%f" % (position[0], position[1]))
            orientation, position = self.yaml_handler.get_pose_for(item.destination_str)
            if position != None:
                item.dest_dist = DistanceToGoalClient.call_serv(position[0], position[1])
                if (item.dest_dist == -1):
                    return False
        return True

    def sort_list_by_distance_src_dest(self, sort_list):
        if (self.update_distances(sort_list)):
            sort_list.task_list.sort(key=cmp_to_key(self.cmp_distance_src_dest))    
            return True
        return False

    def sort_list_by_distance_dest(self, sort_list):
        if (self.update_distances(sort_list)):
            sort_list.task_list.sort(key=cmp_to_key(self.cmp_distance_dest)) 
            return True   
        return False

## ===== TaskTransportOptimizer ===== ##
# Optimizer list of TRANSPORTATION tasks
# Precision placing included 
class TaskTransportOptimizer(object):
    def __init__(self, holding_capacity, yaml_path, verbose=False):
        self.holding_capacity = holding_capacity
        self.input_list = TaskList()
        self.output_list = ActionList()
        self.task_sorter = TaskSorter(yaml_path)
        self.verbose = verbose
    
    def reinitialize_input(self, input):
        self.input_list = input 
        self.output_list.clear_task()

    def prioritize_task_list(self, sort_list):
        if self.verbose: rospy.loginfo("Prioritizing task list...")
        sort_list.sort_by_src_and_dest() # may not be necessary since we're sorting by distances
        self.task_sorter.sort_list_by_distance_src_dest(sort_list)

    def add_task_to_capacity (self, holding_list):
        if not holding_list.is_full():
            (status, index, task) = self.input_list.get_next_obj_to_pick()
            if not status: # picked everything
                return True 
            # check if source changed
            if (len(holding_list.task_list) != 0 and
                task.source != holding_list.task_list[0].source): 
                return True 
            holding_list.add_task(task) 
            self.input_list.task_list[index].status_to_scheduled()
            return False
        else:
            return True
    
    def pick_from_sources (self, task_list):
        unique_sources = task_list.get_unique_source()

        if (len(unique_sources) > 0):
            for key, value in unique_sources.items():
                # Drive to location (included the empty call to move arm)
                TaskFormatter.format_drive(key, self.output_list.task_list)      
                #  List of items to pick up there         
                pick_up = task_list.get_tasks_by_source(key)   
                # Schedule a pick 
                for task in pick_up.task_list:
                    if task.is_dest_same_as_src():
                        # Pick and place right back
                        TaskFormatter.format_pick_task(task, self.output_list.task_list)
                        TaskFormatter.format_place_task(task, self.output_list.task_list)
                        # Remove it from list so a drop-off will not be scheduled
                        self.input_list.remove_task(task) 
                    else:
                        # Pick and place to robot
                        TaskFormatter.format_pick_task(task, self.output_list.task_list)
                        TaskFormatter.format_place_to_robot(task, self.output_list.task_list)

    def drop_off_precision (self, drop_off_list):
        for task in drop_off_list.task_list:
            # Look for the hole
            TaskFormatter.format_find_hole(task, self.output_list.task_list)
            # Pick and place as usual
            TaskFormatter.format_pick_from_robot(task, self.output_list.task_list)
            TaskFormatter.format_place_task(task, self.output_list.task_list)

    def drop_off_normal (self, drop_off_list):   
        for task in drop_off_list.task_list:
            TaskFormatter.format_pick_from_robot(task, self.output_list.task_list)
            TaskFormatter.format_place_task(task, self.output_list.task_list) 

    def drop_off_at_destinations (self, task_list):
        unique_destinations = task_list.get_unique_destination()

        if len(unique_destinations) > 0:
            for key, value in unique_destinations.items():
                # Drive to location (included the empty call to move arm)
                TaskFormatter.format_drive(key, self.output_list.task_list)         
                # List of all items to drop off there   
                drop_off_list = task_list.get_tasks_by_destination(key)   
                # Schedule a drop off
                # LocationIdentifierType.PP = Type of the precision platform
                if (value == LocationIdentifierType.PP.fullname):
                    if self.verbose: rospy.loginfo("Drop off precision-style")
                    self.drop_off_precision(drop_off_list)
                else:
                    if self.verbose: rospy.loginfo("Regular drop off")
                    self.drop_off_normal(drop_off_list)

    def optimize(self):
        # Clear things
        holding_list = TaskList()
        holding_list_ok = False

        self.prioritize_task_list(self.input_list)
        if self.verbose: print(self.input_list)

        # While there is still tasks to schedule
        while not self.input_list.is_empty():
            holding_list_ok = self.add_task_to_capacity(holding_list)

            if (holding_list_ok):
                # Schedule pick from the current source
                self.pick_from_sources(holding_list)
                # Schedule drop off for those items
                self.drop_off_at_destinations(holding_list)

                # Clean up holding list
                self.input_list.remove_task_in_list(holding_list.task_list)
                holding_list.clear_task()
                holding_list_ok = False # Reset flag
        
        # Check for any remaining items
        if not holding_list.is_empty():
            # Schedule pick from the current source
            self.pick_from_sources(holding_list)
            # Schedule drop off for those items
            self.drop_off_at_destinations(holding_list)

        # Clean up holding list
        self.input_list.remove_task_in_list(holding_list.task_list)
        holding_list.clear_task()

        return True # well i should hope it doesn't error

## ===== TaskTransportOptimizer ===== ##
# Optimizer list of NAVIGATION tasks
class TaskNavigationOptimizer(object):
    def __init__(self, yaml_path, verbose=False):
        self.input_list = TaskList()
        self.output_list = ActionList()
        self.task_sorter = TaskSorter(yaml_path)
        self.verbose = verbose

    def reinitialize_input(self, input):
        self.input_list = input
        self.output_list.clear_task()

    def prioritize_task_list(self, sort_list):
        if self.verbose: rospy.loginfo("Prioritizing task list...")
        return self.task_sorter.sort_list_by_distance_dest(sort_list)

    def optimize(self):
        if not self.prioritize_task_list(self.input_list):
            rospy.logerr("Failed to sort by distance. Check 'get_distance' service.")
        if self.verbose: print(self.input_list)
        for item in self.input_list.task_list:
            TaskFormatter.format_drive(item.destination, self.output_list.task_list)
        return True

## ===== TaskReplanner ===== ##
# Output a new TaskList() given the error list
# For TaskManager to make a new ActionList() 
class TaskReplanner(object):
    def __init__(self, yaml_path, verbose=False):
        self.input_list = ActionList()
        self.original_task_list = None
        self.output_list = TaskList()
        self.task_sorter = TaskSorter(yaml_path)
        self.error_index = -1
        self.error_case = -1
        self.verbose = verbose

    def reinitialize_input(self, input, original_list, error_index):
        self.input_list = input
        self.original_task_list = original_list
        self.error_index = error_index
        self.output_list.clear_task()
        self.error_case = None

    def check_error_case(self, task):
        if (task.action == int(TaskActionType.DRIVE)):
            return ErrorCase.CANNOT_DRIVE_TO_LOC

    def replan(self):
        self.error_case = self.check_error_case(self.input_list.task_list[self.error_index])
        self.output_list = self.original_task_list.make_duplicate()
        rospy.logwarn(self.error_case)

        if (self.error_case == ErrorCase.CANNOT_DRIVE_TO_LOC):
            if self.verbose:
                rospy.logwarn('Cannot drive to location ID %d - %s' % (self.input_list.task_list[self.error_index].destination, self.input_list.task_list[self.error_index].destination_str))
                rospy.logwarn('Removing all tasks related to that location...')
            self.remove_all_tasks_with_loc(self.input_list.task_list[self.error_index].destination)
            if self.verbose:
                rospy.loginfo("Task list after removing location")
                print(self.output_list)
        return True

    def remove_all_tasks_with_loc(self, destID):
        src_tasks = self.output_list.get_tasks_by_source(destID)
        dest_tasks = self.output_list.get_tasks_by_destination(destID)
        self.output_list.remove_task_in_list(src_tasks.task_list)
        self.output_list.remove_task_in_list(dest_tasks.task_list)

